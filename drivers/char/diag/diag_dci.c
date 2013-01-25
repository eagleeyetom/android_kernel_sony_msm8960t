/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/diagchar.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <asm/current.h>
#ifdef CONFIG_DIAG_OVER_USB
#include <mach/usbdiag.h>
#endif
#include "diagchar_hdlc.h"
#include "diagmem.h"
#include "diagchar.h"
#include "diagfwd.h"
#include "diag_dci.h"

unsigned int dci_max_reg = 100;
unsigned int dci_max_clients = 10;
unsigned char dci_cumulative_log_mask[DCI_LOG_MASK_SIZE];
unsigned char dci_cumulative_event_mask[DCI_EVENT_MASK_SIZE];
struct mutex dci_log_mask_mutex;
struct mutex dci_event_mask_mutex;

#define DCI_CHK_CAPACITY(entry, new_data_len)				\
((entry->data_len + new_data_len > entry->total_capacity) ? 1 : 0)	\

/* Process the data read from the smd dci channel */
int diag_process_smd_dci_read_data(struct diag_smd_info *smd_info, void *buf,
								int recd_bytes)
{
	int read_bytes, dci_pkt_len, i;
	uint8_t recv_pkt_cmd_code;

	/* Each SMD read can have multiple DCI packets */
	read_bytes = 0;
	while (read_bytes < recd_bytes) {
		/* read actual length of dci pkt */
		dci_pkt_len = *(uint16_t *)(buf+2);
		/* process one dci packet */
		pr_debug("diag: bytes read = %d, single dci pkt len = %d\n",
			read_bytes, dci_pkt_len);
		/* print_hex_dump(KERN_DEBUG, "Single DCI packet :",
		 DUMP_PREFIX_ADDRESS, 16, 1, buf, 5 + dci_pkt_len, 1);*/
		recv_pkt_cmd_code = *(uint8_t *)(buf+4);
		if (recv_pkt_cmd_code == LOG_CMD_CODE)
			extract_dci_log(buf+4);
		else if (recv_pkt_cmd_code == EVENT_CMD_CODE)
			extract_dci_events(buf+4);
		else
			extract_dci_pkt_rsp(buf); /* pkt response */
		read_bytes += 5 + dci_pkt_len;
		buf += 5 + dci_pkt_len; /* advance to next DCI pkt */
	}
	/* wake up all sleeping DCI clients which have some data */
	for (i = 0; i < MAX_DCI_CLIENTS; i++) {
		if (driver->dci_client_tbl[i].client &&
			driver->dci_client_tbl[i].data_len) {
			smd_info->in_busy_1 = 1;
			diag_update_sleeping_process(
				driver->dci_client_tbl[i].client->tgid,
					 DCI_DATA_TYPE);
		}
	}

	return 0;
}

void extract_dci_pkt_rsp(unsigned char *buf)
{
	int i = 0, index = -1, cmd_code_len = 1;
	int curr_client_pid = 0, write_len;
	struct diag_dci_client_tbl *entry;
	void *temp_buf = NULL;
	uint8_t recv_pkt_cmd_code;

	recv_pkt_cmd_code = *(uint8_t *)(buf+4);
	if (recv_pkt_cmd_code != DCI_PKT_RSP_CODE)
		cmd_code_len = 4; /* delayed response */
	write_len = (int)(*(uint16_t *)(buf+2)) - cmd_code_len;
	pr_debug("diag: len = %d\n", write_len);
	/* look up DCI client with tag */
	for (i = 0; i < dci_max_reg; i++) {
		if (driver->req_tracking_tbl[i].tag ==
					 *(int *)(buf+(4+cmd_code_len))) {
			*(int *)(buf+4+cmd_code_len) =
					driver->req_tracking_tbl[i].uid;
			curr_client_pid =
					 driver->req_tracking_tbl[i].pid;
			index = i;
			break;
		}
	}
	if (index == -1)
		pr_alert("diag: No matching PID for DCI data\n");
	/* Using PID of client process, find client buffer */
	for (i = 0; i < MAX_DCI_CLIENTS; i++) {
		if (driver->dci_client_tbl[i].client != NULL) {
			if (curr_client_pid ==
				driver->dci_client_tbl[i].client->tgid) {
				/* copy pkt rsp in client buf */
				entry = &(driver->dci_client_tbl[i]);
				if (DCI_CHK_CAPACITY(entry, 8+write_len)) {
					pr_alert("diag: create capacity for pkt rsp\n");
					entry->total_capacity += 8+write_len;
					temp_buf = krealloc(entry->dci_data,
					entry->total_capacity, GFP_KERNEL);
					if (!temp_buf) {
						pr_err("diag: DCI realloc failed\n");
						break;
					} else {
						entry->dci_data = temp_buf;
					}
				}
				*(int *)(entry->dci_data+entry->data_len) =
							DCI_PKT_RSP_TYPE;
				entry->data_len += 4;
				*(int *)(entry->dci_data+entry->data_len)
								= write_len;
				entry->data_len += 4;
				memcpy(entry->dci_data+entry->data_len,
					buf+4+cmd_code_len, write_len);
				entry->data_len += write_len;
				/* delete immediate response entry */
				if (driver->smd_dci[MODEM_DATA].
					buf_in_1[8+cmd_code_len] != 0x80)
					driver->req_tracking_tbl[index].pid = 0;
				break;
			}
		}
		if (found)
			pr_alert("diag: No matching PID for DCI data\n");
		pr_debug("\n diag PID = %d", driver->dci_tbl[i].pid);
		if (driver->dci_tbl[i].pid == 0)
			pr_alert("diag: Receiving DCI process deleted\n");
		*(int *)(buf+4+cmd_code_len) = driver->dci_tbl[i].uid;
		/* update len after adding UID */
		driver->write_ptr_dci->length =
			driver->write_ptr_dci->length + 4;
		pr_debug("diag: data receivd, wake process\n");
		driver->in_busy_dci = 1;
		diag_update_sleeping_process(driver->dci_tbl[i].pid,
							DCI_DATA_TYPE);
		/* delete immediate response entry */
		if (driver->buf_in_dci[8+cmd_code_len] != 0x80)
			driver->dci_tbl[i].pid = 0;
		for (i = 0; i < dci_max_reg; i++)
			if (driver->dci_tbl[i].pid != 0)
				pr_debug("diag: PID = %d, UID = %d, tag = %d\n",
				driver->dci_tbl[i].pid, driver->dci_tbl[i].uid,
				 driver->dci_tbl[i].tag);
		pr_debug("diag: completed clearing table\n");
	}
}

void extract_dci_events(unsigned char *buf)
{
	uint16_t event_id, event_id_packet;
	uint8_t *event_mask_ptr, byte_mask, payload_len;
	uint8_t event_data[MAX_EVENT_SIZE], timestamp[8];
	int i, byte_index, bit_index, length, temp_len;
	int total_event_len, payload_len_field, timestamp_len;
	struct diag_dci_client_tbl *entry;

	length =  *(uint16_t *)(buf+1); /* total length of event series */
	temp_len = 0;
	buf = buf + 3; /* start of event series */
	while (temp_len < length-1) {
		*event_data = EVENT_CMD_CODE;
		event_id_packet = *(uint16_t *)(buf+temp_len);
		event_id = event_id_packet & 0x0FFF; /* extract 12 bits */
		if (event_id_packet & 0x8000) {
			timestamp_len = 2;
		} else {
			timestamp_len = 8;
			memcpy(timestamp, buf+temp_len+2, 8);
		}
		if (((event_id_packet & 0x6000) >> 13) == 3) {
			payload_len_field = 1;
			payload_len = *(uint8_t *)
					(buf+temp_len+2+timestamp_len);
			memcpy(event_data+13, buf+temp_len+2+timestamp_len, 1);
			memcpy(event_data+14, buf+temp_len+2+timestamp_len+1,
								 payload_len);
		} else {
			payload_len_field = 0;
			payload_len = (event_id_packet & 0x6000) >> 13;
			if (payload_len < MAX_EVENT_SIZE)
				memcpy(event_data+13,
				 buf+temp_len+2+timestamp_len, payload_len);
			else
				pr_alert("diag: event > %d\n", MAX_EVENT_SIZE);
		}
		/* 2 bytes for the event id & timestamp len is hard coded to 8,
		   as individual events have full timestamp */
		*(uint16_t *)(event_data+1) = 10+payload_len_field+payload_len;
		*(uint16_t *)(event_data+3) = event_id_packet & 0x7FFF;
		memcpy(event_data+5, timestamp, 8);
		total_event_len = 3 + 10 + payload_len_field + payload_len;
		byte_index = event_id/8;
		bit_index = event_id % 8;
		byte_mask = 0x1 << bit_index;
		/* parse through event mask tbl of each client and check mask */
		for (i = 0; i < MAX_DCI_CLIENTS; i++) {
			if (driver->dci_client_tbl[i].client) {
				entry = &(driver->dci_client_tbl[i]);
				event_mask_ptr = entry->dci_event_mask +
								 byte_index;
				if (*event_mask_ptr & byte_mask) {
					/* copy to client buffer */
					if (DCI_CHK_CAPACITY(entry,
							 4 + total_event_len)) {
						pr_err("diag:DCI event drop\n");
						driver->dci_client_tbl[i].
							dropped_events++;
						return;
					}
					driver->dci_client_tbl[i].
							received_events++;
					*(int *)(entry->dci_data+
					entry->data_len) = DCI_EVENT_TYPE;
					memcpy(entry->dci_data+
				entry->data_len+4, event_data, total_event_len);
					entry->data_len += 4 + total_event_len;
				}
			}
		}
		temp_len += 2 + timestamp_len + payload_len_field + payload_len;
	}
}

void extract_dci_log(unsigned char *buf)
{
	uint16_t log_code, item_num;
	uint8_t equip_id, *log_mask_ptr, byte_mask;
	int i, byte_index, found = 0;
	struct diag_dci_client_tbl *entry;

	log_code = *(uint16_t *)(buf+6);
	equip_id = LOG_GET_EQUIP_ID(log_code);
	item_num = LOG_GET_ITEM_NUM(log_code);
	byte_index = item_num/8 + 2;
	byte_mask = 0x01 << (item_num % 8);

	/* parse through log mask table of each client and check mask */
	for (i = 0; i < MAX_DCI_CLIENTS; i++) {
		if (driver->dci_client_tbl[i].client) {
			entry = &(driver->dci_client_tbl[i]);
			log_mask_ptr = entry->dci_log_mask;
			found = 0;
			while (log_mask_ptr) {
				if (*log_mask_ptr == equip_id) {
					found = 1;
					pr_debug("diag: find equip id = %x at %p\n",
					equip_id, log_mask_ptr);
					break;
				} else {
					pr_debug("diag: did not find equip id = %x at %p\n",
						 equip_id, log_mask_ptr);
					log_mask_ptr += 514;
				}
			}
			if (!found)
				pr_err("diag: dci equip id not found\n");
			log_mask_ptr = log_mask_ptr + byte_index;
			if (*log_mask_ptr & byte_mask) {
				pr_debug("\t log code %x needed by client %d",
					 log_code, entry->client->tgid);
				/* copy to client buffer */
				if (DCI_CHK_CAPACITY(entry,
						 4 + *(uint16_t *)(buf+2))) {
						pr_err("diag:DCI log drop\n");
						driver->dci_client_tbl[i].
								dropped_logs++;
						return;
				}
				driver->dci_client_tbl[i].received_logs++;
				*(int *)(entry->dci_data+entry->data_len) =
								DCI_LOG_TYPE;
				memcpy(entry->dci_data+entry->data_len+4, buf+4,
						 *(uint16_t *)(buf+2));
				entry->data_len += 4 + *(uint16_t *)(buf+2);
			}
		}
	}
}

void diag_update_smd_dci_work_fn(struct work_struct *work)
{
	struct diag_smd_info *smd_info = container_of(work,
						struct diag_smd_info,
						diag_notify_update_smd_work);
	int i, j;
	char dirty_bits[16];
	uint8_t *client_log_mask_ptr;
	uint8_t *log_mask_ptr;
	int ret;
	int index = smd_info->peripheral;

	/* Update the peripheral(s) with the dci log and event masks */

	/* If the cntl channel is not up, we can't update logs and events */
	if (!driver->smd_cntl[index].ch)
		return;

	memset(dirty_bits, 0, 16 * sizeof(uint8_t));

	/*
	 * From each log entry used by each client, determine
	 * which log entries in the cumulative logs that need
	 * to be updated on the peripheral.
	 */
	for (i = 0; i < MAX_DCI_CLIENTS; i++) {
		if (driver->dci_client_tbl[i].client) {
			client_log_mask_ptr =
				driver->dci_client_tbl[i].dci_log_mask;
			for (j = 0; j < 16; j++) {
				if (*(client_log_mask_ptr+1))
					dirty_bits[j] = 1;
				client_log_mask_ptr += 514;
			}
		}
	}

	mutex_lock(&dci_log_mask_mutex);
	/* Update the appropriate dirty bits in the cumulative mask */
	log_mask_ptr = dci_cumulative_log_mask;
	for (i = 0; i < 16; i++) {
		if (dirty_bits[i])
			*(log_mask_ptr+1) = dirty_bits[i];

		log_mask_ptr += 514;
	}
	mutex_unlock(&dci_log_mask_mutex);

	ret = diag_send_dci_log_mask(driver->smd_cntl[index].ch);

	ret = diag_send_dci_event_mask(driver->smd_cntl[index].ch);

	smd_info->notify_context = 0;
}

void diag_dci_notify_client(int peripheral_mask)
{
	int i, stat;

	/* Notify the DCI process that the peripheral DCI Channel is up */
	for (i = 0; i < MAX_DCI_CLIENT; i++) {
		if (driver->dci_notify_tbl[i].list & peripheral_mask) {
			pr_debug("diag: sending signal now\n");
			stat = send_sig(driver->dci_notify_tbl[i].signal_type,
					 driver->dci_notify_tbl[i].client, 0);
			if (stat)
				pr_err("diag: Err send sig stat: %d\n", stat);
			break;
		}
	} /* end of loop for all DCI clients */
}

static int diag_dci_probe(struct platform_device *pdev)
{
	int err = 0;
	int index;

	if (pdev->id == SMD_APPS_MODEM) {
		index = MODEM_DATA;
		err = smd_open("DIAG_2", &driver->smd_dci[index].ch,
					&driver->smd_dci[index],
					diag_smd_notify);
		driver->smd_dci[index].ch_save =
					driver->smd_dci[index].ch;
		if (err)
			pr_err("diag: In %s, cannot open DCI port, Id = %d, err: %d\n",
				__func__, pdev->id, err);
	}

	return err;
}


int diag_send_dci_pkt(struct diag_master_table entry, unsigned char *buf,
					 int len, int index)
{
	int i;
	int status = 0;

	/* remove UID from user space pkt before sending to peripheral */
	buf = buf + 4;
	len = len - 4;
	mutex_lock(&driver->dci_mutex);
	/* prepare DCI packet */
	driver->apps_dci_buf[0] = CONTROL_CHAR; /* start */
	driver->apps_dci_buf[1] = 1; /* version */
	*(uint16_t *)(driver->apps_dci_buf + 2) = len + 4 + 1; /* length */
	driver->apps_dci_buf[4] = DCI_CMD_CODE; /* DCI ID */
	*(int *)(driver->apps_dci_buf + 5) = driver->dci_tbl[index].tag;
	for (i = 0; i < len; i++)
		driver->apps_dci_buf[i+9] = *(buf+i);
	driver->apps_dci_buf[9+len] = CONTROL_CHAR; /* end */

	for (i = 0; i < NUM_SMD_DCI_CHANNELS; i++) {
		if (entry.client_id == driver->smd_dci[i].peripheral) {
			if (driver->smd_dci[i].ch) {
				smd_write(driver->smd_dci[i].ch,
					driver->apps_dci_buf, len + 10);
				status = DIAG_DCI_NO_ERROR;
			}
			break;
		}
	}

	if (status != DIAG_DCI_NO_ERROR) {
		pr_alert("diag: check DCI channel\n");
		status = DIAG_DCI_SEND_DATA_FAIL;
	}
	mutex_unlock(&driver->dci_mutex);
	return status;
}

int diag_register_dci_transaction(int uid)
{
	int i, new_dci_client = 1, ret = -1;

	for (i = 0; i < dci_max_reg; i++) {
		if (driver->dci_tbl[i].pid == current->tgid) {
			new_dci_client = 0;
			break;
		}
	}
	mutex_lock(&driver->dci_mutex);
	if (new_dci_client)
		driver->num_dci_client++;
	if (driver->num_dci_client > MAX_DCI_CLIENT) {
		pr_info("diag: Max DCI Client limit reached\n");
		driver->num_dci_client--;
		mutex_unlock(&driver->dci_mutex);
		return ret;
	}
	/* Make an entry in kernel DCI table */
	driver->dci_tag++;
	for (i = 0; i < dci_max_reg; i++) {
		if (driver->dci_tbl[i].pid == 0) {
			driver->dci_tbl[i].pid = current->tgid;
			driver->dci_tbl[i].uid = uid;
			driver->dci_tbl[i].tag = driver->dci_tag;
			ret = i;
			break;
		}
	}
	mutex_unlock(&driver->dci_mutex);
	return ret;
}

int diag_process_dci_client(unsigned char *buf, int len)
{
	unsigned char *temp = buf;
	uint16_t subsys_cmd_code;
	int subsys_id, cmd_code, i, ret = -1, index = -1;
	struct diag_master_table entry;

	if (!driver->smd_dci[MODEM_DATA].ch) {
		pr_err("diag: DCI smd channel for peripheral %d not valid for dci updates\n",
			driver->smd_dci[MODEM_DATA].peripheral);
		return DIAG_DCI_SEND_DATA_FAIL;
	}
	temp += 4;
	/* Check for registered peripheral and fwd pkt to apropriate proc */
	cmd_code = (int)(*(char *)buf);
	temp++;
	subsys_id = (int)(*(char *)temp);
	temp++;
	subsys_cmd_code = *(uint16_t *)temp;
	temp += 2;
	pr_debug("diag: %d %d %d", cmd_code, subsys_id, subsys_cmd_code);
	for (i = 0; i < diag_max_reg; i++) {
		entry = driver->table[i];
		if (entry.process_id != NO_PROCESS) {
			if (entry.cmd_code == cmd_code && entry.subsys_id ==
				 subsys_id && entry.cmd_code_lo <=
							 subsys_cmd_code &&
				  entry.cmd_code_hi >= subsys_cmd_code) {
				ret = diag_send_dci_pkt(entry, buf, len, index);
			} else if (entry.cmd_code == 255
				  && cmd_code == 75) {
				if (entry.subsys_id ==
					subsys_id &&
				   entry.cmd_code_lo <=
					subsys_cmd_code &&
					 entry.cmd_code_hi >=
					subsys_cmd_code) {
					ret = diag_send_dci_pkt(entry, buf, len,
								 index);
				}
			} else if (entry.cmd_code == 255 &&
				  entry.subsys_id == 255) {
				if (entry.cmd_code_lo <=
						 cmd_code &&
						 entry.
						cmd_code_hi >= cmd_code) {
					ret = diag_send_dci_pkt(entry, buf, len,
								 index);
				}
			}
			if (!found) {
				pr_err("diag: dci equip id not found\n");
				return ret;
			}
			*(log_mask_ptr+1) = 1; /* set the dirty byte */
			log_mask_ptr = log_mask_ptr + byte_index;
			if (set_mask)
				*log_mask_ptr |= byte_mask;
			else
				*log_mask_ptr &= ~byte_mask;
			/* add to cumulative mask */
			update_dci_cumulative_log_mask(
				offset, byte_index,
				byte_mask);
			temp += 2;
			count++;
			ret = DIAG_DCI_NO_ERROR;
		}
		/* send updated mask to peripherals */
		ret = diag_send_dci_log_mask(driver->smd_cntl[MODEM_DATA].ch);
	} else if (*(int *)temp == DCI_EVENT_TYPE) {
		/* find client id and table */
		for (i = 0; i < MAX_DCI_CLIENTS; i++) {
			if (driver->dci_client_tbl[i].client != NULL) {
				if (driver->dci_client_tbl[i].client->tgid ==
							current->tgid) {
					found = 1;
					break;
				}
			}
		}
		if (!found) {
			pr_err("diag: dci client not registered/found\n");
			return ret;
		}
		/* Extract each log code and put in client table */
		temp += 4;
		set_mask = *(int *)temp;
		temp += 4;
		num_codes = *(int *)temp;
		temp += 4;

		event_mask_ptr = driver->dci_client_tbl[i].dci_event_mask;
		pr_debug("diag: head of dci event mask %p\n", event_mask_ptr);
		count = 0; /* iterator for extracting log codes */
		while (count < num_codes) {
			event_id = *(int *)temp;
			byte_index = event_id/8;
			bit_index = event_id % 8;
			byte_mask = 0x1 << bit_index;
			/*
			 * Parse through event mask table and set
			 * relevant byte & bit combination
			 */
			if (set_mask)
				*(event_mask_ptr + byte_index) |= byte_mask;
			else
				*(event_mask_ptr + byte_index) &= ~byte_mask;
			/* add to cumulative mask */
			update_dci_cumulative_event_mask(byte_index, byte_mask);
			temp += sizeof(int);
			count++;
			ret = DIAG_DCI_NO_ERROR;
		}
		/* send updated mask to peripherals */
		ret = diag_send_dci_event_mask(driver->smd_cntl[MODEM_DATA].ch);
	} else {
		pr_alert("diag: Incorrect DCI transaction\n");
	}
	return ret;
}

static int diag_dci_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diag_dci_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diag_dci_dev_pm_ops = {
	.runtime_suspend = diag_dci_runtime_suspend,
	.runtime_resume = diag_dci_runtime_resume,
};

struct platform_driver msm_diag_dci_driver = {
	.probe = diag_dci_probe,
	.driver = {
		   .name = "DIAG_2",
		   .owner = THIS_MODULE,
		   .pm   = &diag_dci_dev_pm_ops,
		   },
};

int diag_dci_init(void)
{
	int success = 0;
	int i;

	driver->dci_tag = 0;
	driver->dci_client_id = 0;
	driver->num_dci_client = 0;
	mutex_init(&driver->dci_mutex);
	mutex_init(&dci_log_mask_mutex);
	mutex_init(&dci_event_mask_mutex);
	success = diag_smd_constructor(&driver->smd_dci[MODEM_DATA],
					MODEM_DATA, SMD_DCI_TYPE);
	if (!success)
		goto err;

	if (driver->req_tracking_tbl == NULL) {
		driver->req_tracking_tbl = kzalloc(dci_max_reg *
			sizeof(struct dci_pkt_req_tracking_tbl), GFP_KERNEL);
		if (driver->req_tracking_tbl == NULL)
			goto err;
	}
	if (driver->apps_dci_buf == NULL) {
		driver->apps_dci_buf = kzalloc(APPS_BUF_SIZE, GFP_KERNEL);
		if (driver->apps_dci_buf == NULL)
			goto err;
	}
	success = platform_driver_register(&msm_diag_dci_driver);
	if (success) {
		pr_err("diag: Could not register DCI driver\n");
		goto err;
	}
	return DIAG_DCI_NO_ERROR;
err:
	pr_err("diag: Could not initialize diag DCI buffers");
	kfree(driver->dci_tbl);
	kfree(driver->dci_notify_tbl);
	kfree(driver->apps_dci_buf);
	for (i = 0; i < NUM_SMD_DCI_CHANNELS; i++)
		diag_smd_destructor(&driver->smd_dci[i]);
	if (driver->diag_dci_wq)
		destroy_workqueue(driver->diag_dci_wq);
	return DIAG_DCI_NO_REG;
}

void diag_dci_exit(void)
{
	int i;

	for (i = 0; i < NUM_SMD_DCI_CHANNELS; i++)
		diag_smd_destructor(&driver->smd_dci[i]);

	platform_driver_unregister(&msm_diag_dci_driver);
	kfree(driver->dci_tbl);
	kfree(driver->dci_notify_tbl);
	kfree(driver->apps_dci_buf);
	destroy_workqueue(driver->diag_dci_wq);
}

