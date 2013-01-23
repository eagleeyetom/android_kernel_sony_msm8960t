/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/slab.h>
#include <linux/diagchar.h>
#include <linux/platform_device.h>
#include <linux/kmemleak.h>
#include "diagchar.h"
#include "diagfwd.h"
#include "diagfwd_cntl.h"
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
/* tracks which peripheral is undergoing SSR */
static uint16_t reg_dirty;
#define HDR_SIZ 8

void diag_clean_reg_fn(struct work_struct *work)
{
	struct diag_smd_info *smd_info = container_of(work,
						struct diag_smd_info,
						diag_notify_update_smd_work);
	if (!smd_info)
		return;

	pr_debug("diag: clean registration for peripheral: %d\n",
		smd_info->peripheral);

	reg_dirty |= smd_info->peripheral_mask;
	diag_clear_reg(smd_info->peripheral);
	reg_dirty ^= smd_info->peripheral_mask;

	smd_info->notify_context = 0;
}

/* Process the data read from the smd control channel */
int diag_process_smd_cntl_read_data(struct diag_smd_info *smd_info, void *buf,
								int total_recd)
{
	int data_len = 0, type = -1, count_bytes = 0, j, flag = 0;
	int feature_mask_len;
	struct bindpkt_params_per_process *pkt_params =
		kzalloc(sizeof(struct bindpkt_params_per_process), GFP_KERNEL);
	struct diag_ctrl_msg *msg;
	struct cmd_code_range *range;
	struct bindpkt_params *temp;

	if (pkt_params == NULL) {
		pr_alert("diag: In %s, Memory allocation failure\n",
			__func__);
		return 0;
	}

	if (!smd_info) {
		pr_err("diag: In %s, No smd info. Not able to read.\n",
			__func__);
		kfree(pkt_params);
		return 0;
	}

	while (count_bytes + HDR_SIZ <= total_recd) {
		type = *(uint32_t *)(buf);
		data_len = *(uint32_t *)(buf + 4);
		if (type < DIAG_CTRL_MSG_REG ||
				 type > DIAG_CTRL_MSG_F3_MASK_V2) {
			pr_alert("diag: In %s, Invalid Msg type %d proc %d",
				 __func__, type, smd_info->peripheral);
			break;
		}
		if (data_len < 0 || data_len > total_recd) {
			pr_alert("diag: In %s, Invalid data len %d, total_recd: %d, proc %d",
				 __func__, data_len, total_recd,
				 smd_info->peripheral);
			break;
		}
		count_bytes = count_bytes+HDR_SIZ+data_len;
		if (type == DIAG_CTRL_MSG_REG && total_recd >= count_bytes) {
			msg = buf+HDR_SIZ;
			range = buf+HDR_SIZ+
					sizeof(struct diag_ctrl_msg);
			pkt_params->count = msg->count_entries;
			temp = kzalloc(pkt_params->count * sizeof(struct
					 bindpkt_params), GFP_KERNEL);
			if (temp == NULL) {
				pr_alert("diag: In %s, Memory alloc fail\n",
					__func__);
				kfree(pkt_params);
				return flag;
			}
			for (j = 0; j < pkt_params->count; j++) {
				temp->cmd_code = msg->cmd_code;
				temp->subsys_id = msg->subsysid;
				temp->client_id = smd_info->peripheral;
				temp->proc_id = NON_APPS_PROC;
				temp->cmd_code_lo = range->cmd_code_lo;
				temp->cmd_code_hi = range->cmd_code_hi;
				range++;
				temp++;
			}
			temp -= pkt_params->count;
			pkt_params->params = temp;
			flag = 1;
			/* peripheral undergoing SSR should not
			 * record new registration
			 */
			if (!(reg_dirty & smd_info->peripheral_mask))
				diagchar_ioctl(NULL, DIAG_IOCTL_COMMAND_REG,
						(unsigned long)pkt_params);
			else
				pr_err("diag: drop reg proc %d\n",
						smd_info->peripheral);
			kfree(temp);
		} else if ((type == DIAG_CTRL_MSG_FEATURE) &&
				(smd_info->peripheral == MODEM_DATA)) {
			feature_mask_len = *(int *)(buf + 8);
			driver->log_on_demand_support = (*(uint8_t *)
							 (buf + 12)) & 0x04;
		} else if (type != DIAG_CTRL_MSG_REG) {
			flag = 1;
		}
		buf = buf + HDR_SIZ + data_len;
	}
	kfree(pkt_params);

	return flag;
}

static int diag_smd_cntl_probe(struct platform_device *pdev)
{
	int r = 0;
	int index = -1;

	/* open control ports only on 8960 & newer targets */
	if (chk_apps_only()) {
		if (pdev->id == SMD_APPS_MODEM) {
			index = MODEM_DATA;
			r = smd_open("DIAG_CNTL",
					&driver->smd_cntl[index].ch,
					&driver->smd_cntl[index],
					diag_smd_notify);
			driver->smd_cntl[index].ch_save =
					driver->smd_cntl[index].ch;
		} else if (pdev->id == SMD_APPS_QDSP) {
			index = LPASS_DATA;
			r = smd_named_open_on_edge("DIAG_CNTL",
					SMD_APPS_QDSP,
					&driver->smd_cntl[index].ch,
					&driver->smd_cntl[index],
					diag_smd_notify);
			driver->smd_cntl[index].ch_save =
					driver->smd_cntl[index].ch;
		} else if (pdev->id == SMD_APPS_WCNSS) {
			index = WCNSS_DATA;
			r = smd_named_open_on_edge("APPS_RIVA_CTRL",
					SMD_APPS_WCNSS,
					&driver->smd_cntl[index].ch,
					&driver->smd_cntl[index],
					diag_smd_notify);
			driver->smd_cntl[index].ch_save =
					driver->smd_cntl[index].ch;
		}

		pr_debug("diag: open CNTL port, ID = %d,r = %d\n", pdev->id, r);
	}
	return 0;
}

static int diagfwd_cntl_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diagfwd_cntl_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diagfwd_cntl_dev_pm_ops = {
	.runtime_suspend = diagfwd_cntl_runtime_suspend,
	.runtime_resume = diagfwd_cntl_runtime_resume,
};

static struct platform_driver msm_smd_ch1_cntl_driver = {

	.probe = diag_smd_cntl_probe,
	.driver = {
			.name = "DIAG_CNTL",
			.owner = THIS_MODULE,
			.pm   = &diagfwd_cntl_dev_pm_ops,
		   },
};

static struct platform_driver diag_smd_lite_cntl_driver = {

	.probe = diag_smd_cntl_probe,
	.driver = {
			.name = "APPS_RIVA_CTRL",
			.owner = THIS_MODULE,
			.pm   = &diagfwd_cntl_dev_pm_ops,
		   },
};

void diagfwd_cntl_init(void)
{
	int success;
	int i;

	reg_dirty = 0;
	driver->polling_reg_flag = 0;
	driver->log_on_demand_support = 1;
	driver->diag_cntl_wq = create_singlethread_workqueue("diag_cntl_wq");

	success = diag_smd_constructor(&driver->smd_cntl[MODEM_DATA],
					MODEM_DATA, SMD_CNTL_TYPE);
	if (!success)
		goto err;

	success = diag_smd_constructor(&driver->smd_cntl[LPASS_DATA],
					LPASS_DATA, SMD_CNTL_TYPE);
	if (!success)
		goto err;

	success = diag_smd_constructor(&driver->smd_cntl[WCNSS_DATA],
					WCNSS_DATA, SMD_CNTL_TYPE);
	if (!success)
		goto err;

	platform_driver_register(&msm_smd_ch1_cntl_driver);
	platform_driver_register(&diag_smd_lite_cntl_driver);

	return;
err:
	pr_err("diag: Could not initialize diag buffers");

	for (i = 0; i < NUM_SMD_CONTROL_CHANNELS; i++)
		diag_smd_destructor(&driver->smd_cntl[i]);

	if (driver->diag_cntl_wq)
		destroy_workqueue(driver->diag_cntl_wq);
}

void diagfwd_cntl_exit(void)
{
	int i;

	for (i = 0; i < NUM_SMD_CONTROL_CHANNELS; i++)
		diag_smd_destructor(&driver->smd_cntl[i]);

	destroy_workqueue(driver->diag_cntl_wq);

	platform_driver_unregister(&msm_smd_ch1_cntl_driver);
	platform_driver_unregister(&diag_smd_lite_cntl_driver);
}

#ifdef CONFIG_DEBUG_FS
#define DEBUG_BUF_SIZE	4096
static struct dentry *diag_dbgfs_dent;
static int diag_dbgfs_table_index;

static ssize_t diag_dbgfs_read_status(struct file *file, char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	char *buf;
	int ret;

	buf = kzalloc(sizeof(char) * DEBUG_BUF_SIZE, GFP_KERNEL);
	if (!buf) {
		pr_err("diag: %s, Error allocating memory\n", __func__);
		return -ENOMEM;
	}

	ret = scnprintf(buf, DEBUG_BUF_SIZE,
		"modem ch: 0x%x\n"
		"lpass ch: 0x%x\n"
		"riva ch: 0x%x\n"
		"dci ch: 0x%x\n"
		"modem cntl_ch: 0x%x\n"
		"lpass cntl_ch: 0x%x\n"
		"riva cntl_ch: 0x%x\n"
		"CPU Tools id: %d\n"
		"Apps only: %d\n"
		"Apps master: %d\n"
		"Check Polling Response: %d\n"
		"polling_reg_flag: %d\n"
		"uses device tree: %d\n"
		"in_busy_1: %d\n"
		"in_busy_2: %d\n"
		"in_busy_qdsp_1: %d\n"
		"in_busy_qdsp_2: %d\n"
		"in_busy_wcnss_1: %d\n"
		"in_busy_wcnss_2: %d\n"
		"in_busy_dci: %d\n"
		"logging_mode: %d\n",
		(unsigned int)driver->ch,
		(unsigned int)driver->chqdsp,
		(unsigned int)driver->ch_wcnss,
		(unsigned int)driver->ch_dci,
		(unsigned int)driver->ch_cntl,
		(unsigned int)driver->chqdsp_cntl,
		(unsigned int)driver->ch_wcnss_cntl,
		chk_config_get_id(),
		chk_apps_only(),
		chk_apps_master(),
		chk_polling_response(),
		driver->polling_reg_flag,
		driver->use_device_tree,
		driver->in_busy_1,
		driver->in_busy_2,
		driver->in_busy_qdsp_1,
		driver->in_busy_qdsp_2,
		driver->in_busy_wcnss_1,
		driver->in_busy_wcnss_2,
		driver->in_busy_dci,
		driver->logging_mode);

#ifdef CONFIG_DIAG_OVER_USB
	ret += scnprintf(buf+ret, DEBUG_BUF_SIZE,
		"usb_connected: %d\n",
		driver->usb_connected);
#endif
	ret = simple_read_from_buffer(ubuf, count, ppos, buf, ret);

	kfree(buf);
	return ret;
}

static ssize_t diag_dbgfs_read_workpending(struct file *file,
				char __user *ubuf, size_t count, loff_t *ppos)
{
	char *buf;
	int ret;

	buf = kzalloc(sizeof(char) * DEBUG_BUF_SIZE, GFP_KERNEL);
	if (!buf) {
		pr_err("diag: %s, Error allocating memory\n", __func__);
		return -ENOMEM;
	}

	ret = scnprintf(buf, DEBUG_BUF_SIZE,
		"Pending status for work_stucts:\n"
		"diag_drain_work: %d\n"
		"diag_read_smd_work: %d\n"
		"diag_read_smd_cntl_work: %d\n"
		"diag_read_smd_qdsp_work: %d\n"
		"diag_read_smd_qdsp_cntl_work: %d\n"
		"diag_read_smd_wcnss_work: %d\n"
		"diag_read_smd_wcnss_cntl_work: %d\n"
		"diag_modem_mask_update_work: %d\n"
		"diag_qdsp_mask_update_work: %d\n"
		"diag_wcnss_mask_update_work: %d\n"
		"diag_read_smd_dci_work: %d\n",
		work_pending(&(driver->diag_drain_work)),
		work_pending(&(driver->diag_read_smd_work)),
		work_pending(&(driver->diag_read_smd_cntl_work)),
		work_pending(&(driver->diag_read_smd_qdsp_work)),
		work_pending(&(driver->diag_read_smd_qdsp_cntl_work)),
		work_pending(&(driver->diag_read_smd_wcnss_work)),
		work_pending(&(driver->diag_read_smd_wcnss_cntl_work)),
		work_pending(&(driver->diag_modem_mask_update_work)),
		work_pending(&(driver->diag_qdsp_mask_update_work)),
		work_pending(&(driver->diag_wcnss_mask_update_work)),
		work_pending(&(driver->diag_read_smd_dci_work)));

#ifdef CONFIG_DIAG_OVER_USB
	ret += scnprintf(buf+ret, DEBUG_BUF_SIZE,
		"diag_proc_hdlc_work: %d\n"
		"diag_read_work: %d\n",
		work_pending(&(driver->diag_proc_hdlc_work)),
		work_pending(&(driver->diag_read_work)));
#endif
	ret = simple_read_from_buffer(ubuf, count, ppos, buf, ret);

	kfree(buf);
	return ret;
}

static ssize_t diag_dbgfs_read_table(struct file *file, char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	char *buf;
	int ret = 0;
	int i;
	int bytes_remaining;
	int bytes_in_buffer = 0;
	int bytes_written;
	int buf_size = (DEBUG_BUF_SIZE < count) ? DEBUG_BUF_SIZE : count;

	if (diag_dbgfs_table_index >= diag_max_reg) {
		/* Done. Reset to prepare for future requests */
		diag_dbgfs_table_index = 0;
		return 0;
	}

	buf = kzalloc(sizeof(char) * buf_size, GFP_KERNEL);
	if (!buf) {
		pr_err("diag: %s, Error allocating memory\n", __func__);
		return -ENOMEM;
	}

	bytes_remaining = buf_size;
	for (i = diag_dbgfs_table_index; i < diag_max_reg; i++) {
		/* Do not process empty entries in the table */
		if (driver->table[i].process_id == 0)
			continue;

		bytes_written = scnprintf(buf+bytes_in_buffer, bytes_remaining,
			"i: %3d, cmd_code: %4x, subsys_id: %4x, "
			"client: %2d, cmd_code_lo: %4x, "
			"cmd_code_hi: %4x, process_id: %5d\n",
			i,
			driver->table[i].cmd_code,
			driver->table[i].subsys_id,
			driver->table[i].client_id,
			driver->table[i].cmd_code_lo,
			driver->table[i].cmd_code_hi,
			driver->table[i].process_id);

		bytes_in_buffer += bytes_written;

		/* Check if there is room to add another table entry */
		bytes_remaining = buf_size - bytes_in_buffer;
		if (bytes_remaining < bytes_written)
			break;
	}
	diag_dbgfs_table_index = i;

	*ppos = 0;
	ret = simple_read_from_buffer(ubuf, count, ppos, buf, bytes_in_buffer);

	kfree(buf);
	return ret;
}

#ifdef CONFIG_DIAG_BRIDGE_CODE
static ssize_t diag_dbgfs_read_hsic(struct file *file, char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	char *buf;
	int ret;

	buf = kzalloc(sizeof(char) * DEBUG_BUF_SIZE, GFP_KERNEL);
	if (!buf) {
		pr_err("diag: %s, Error allocating memory\n", __func__);
		return -ENOMEM;
	}

	ret = scnprintf(buf, DEBUG_BUF_SIZE,
		"hsic ch: %d\n"
		"hsic_inited: %d\n"
		"hsic enabled: %d\n"
		"hsic_opened: %d\n"
		"hsic_suspend: %d\n"
		"in_busy_hsic_read_on_device: %d\n"
		"in_busy_hsic_write: %d\n"
		"count_hsic_pool: %d\n"
		"count_hsic_write_pool: %d\n"
		"diag_hsic_pool: %x\n"
		"diag_hsic_write_pool: %x\n"
		"write_len_mdm: %d\n"
		"num_hsic_buf_tbl_entries: %d\n"
		"usb_mdm_connected: %d\n"
		"diag_read_mdm_work: %d\n"
		"diag_read_hsic_work: %d\n"
		"diag_disconnect_work: %d\n"
		"diag_usb_read_complete_work: %d\n",
		driver->hsic_ch,
		driver->hsic_inited,
		driver->hsic_device_enabled,
		driver->hsic_device_opened,
		driver->hsic_suspend,
		driver->in_busy_hsic_read_on_device,
		driver->in_busy_hsic_write,
		driver->count_hsic_pool,
		driver->count_hsic_write_pool,
		(unsigned int)driver->diag_hsic_pool,
		(unsigned int)driver->diag_hsic_write_pool,
		driver->write_len_mdm,
		driver->num_hsic_buf_tbl_entries,
		driver->usb_mdm_connected,
		work_pending(&(driver->diag_read_mdm_work)),
		work_pending(&(driver->diag_read_hsic_work)),
		work_pending(&(driver->diag_disconnect_work)),
		work_pending(&(driver->diag_usb_read_complete_work)));

	ret = simple_read_from_buffer(ubuf, count, ppos, buf, ret);

	kfree(buf);
	return ret;
}

const struct file_operations diag_dbgfs_hsic_ops = {
	.read = diag_dbgfs_read_hsic,
};
#endif

const struct file_operations diag_dbgfs_status_ops = {
	.read = diag_dbgfs_read_status,
};

const struct file_operations diag_dbgfs_table_ops = {
	.read = diag_dbgfs_read_table,
};

const struct file_operations diag_dbgfs_workpending_ops = {
	.read = diag_dbgfs_read_workpending,
};

void diag_debugfs_init(void)
{
	diag_dbgfs_dent = debugfs_create_dir("diag", 0);
	if (IS_ERR(diag_dbgfs_dent))
		return;

	debugfs_create_file("status", 0444, diag_dbgfs_dent, 0,
		&diag_dbgfs_status_ops);

	debugfs_create_file("table", 0444, diag_dbgfs_dent, 0,
		&diag_dbgfs_table_ops);

	debugfs_create_file("work_pending", 0444, diag_dbgfs_dent, 0,
		&diag_dbgfs_workpending_ops);

#ifdef CONFIG_DIAG_BRIDGE_CODE
	debugfs_create_file("hsic", 0444, diag_dbgfs_dent, 0,
		&diag_dbgfs_hsic_ops);
#endif

	diag_dbgfs_table_index = 0;
}

void diag_debugfs_cleanup(void)
{
	if (diag_dbgfs_dent) {
		debugfs_remove_recursive(diag_dbgfs_dent);
		diag_dbgfs_dent = NULL;
	}
}
#else
void diag_debugfs_init(void) { }
void diag_debugfs_cleanup(void) { }
#endif
