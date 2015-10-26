/*
 *  linux/drivers/mmc/host/mmc-transfer-logger.c - MMC transfer logger tool
 */

#include <linux/list.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/mmc/host.h>


struct partition_description {
	unsigned int partition_number;
	unsigned long start;
	unsigned long bytes_written;
	unsigned long bytes_read;
	struct list_head list;
};

struct host_description {
	struct mmc_host *host;
	struct partition_description *partition_list;	
	spinlock_t		logger_lock;		/* Mutex */
	struct list_head list;
};

static struct host_description mmc_host_list = {
	.host = NULL,
	.partition_list = NULL,
	.list = {&mmc_host_list.list, &mmc_host_list.list}
};

static struct host_description* find_host_description(struct mmc_host *host)
{
	struct host_description* entry;
	
	list_for_each_entry(entry, &mmc_host_list.list, list)
	{
		if (entry->host == host)
		{
			return entry;
		}
	}
	return NULL;
}

static void clear_partition_list(struct host_description *host_desc)
{
	struct partition_description* entry;
	struct partition_description* temp_storage;
	
	if (host_desc->partition_list == NULL)
		return;
	
	list_for_each_entry_safe(entry, temp_storage, &host_desc->partition_list->list, list)
	{
		list_del(&entry->list);
		kfree(entry);
	}
}

ssize_t write_logger_show(struct class_device *dev, char * buf)
{
	struct mmc_host *host;
	struct partition_description* entry;
	struct host_description* host_desc;
	ssize_t ret = 0;

	if (dev == NULL)
		return 0;

	host = container_of(dev, struct mmc_host, class_dev);

	host_desc = find_host_description(host);

	if ((host == NULL) || (host_desc == NULL) || (host_desc->partition_list == NULL))
		return 0;

	list_for_each_entry(entry, &host_desc->partition_list->list, list)
	{
		ret += scnprintf(buf + ret, PAGE_SIZE, 
		"Host: %s, Part: %u (Start:%lu), Written: %lu, Read: %lu.\n",
		mmc_hostname(host),entry->partition_number, entry->start,
		entry->bytes_written, entry->bytes_read);
	}

	return ret;
}

static int add_partition_description(const char *buf,
																							struct host_description* host_desc)
{
		struct partition_description* new_entry = 
		kmalloc(sizeof(struct partition_description), GFP_KERNEL);
		
		if (new_entry == NULL)
			return 1;
		
		memset(new_entry,0,sizeof(struct partition_description));
		
		sscanf(buf, "partition %u %lu", &new_entry->partition_number,
		&new_entry->start);
		
		list_add_tail(&new_entry->list, &host_desc->partition_list->list);
		
		return 0;
}

ssize_t write_logger_store(struct class_device *dev, const char * buf, size_t count)
{
	struct mmc_host *host;
	unsigned long logger_flags;
	struct host_description* host_desc;

	if(dev == NULL)
		return 0;

	host = container_of(dev, struct mmc_host, class_dev);

	host_desc = find_host_description(host);
	
	if ((host == NULL) || (host_desc == NULL) || (host_desc->partition_list == NULL))
		return 0;
	
	if (!strncmp(buf,"reset",strlen("reset")))
	{
		spin_lock_irqsave(&host_desc->logger_lock, logger_flags);	
		clear_partition_list(host_desc);
		spin_unlock_irqrestore(&host_desc->logger_lock, logger_flags);
	}
	else
	{
		if (add_partition_description(buf, host_desc))
			return 0;
	}
	return count;
}

CLASS_DEVICE_ATTR(transfer_log, S_IRUGO|S_IWUGO, write_logger_show, write_logger_store);

void transfer_logger_log_transfer(struct mmc_host *host,
	unsigned long start, unsigned int blksz,
	unsigned long bytes_xfered, unsigned int write)
{
	struct partition_description* entry;
	struct host_description* host_desc;
	unsigned long logger_flags;
	
	host_desc = find_host_description(host);
	
	if ((host == NULL) || (host_desc == NULL))
		return;
	
	spin_lock_irqsave(&host_desc->logger_lock, logger_flags);
	
	if (host_desc->partition_list == NULL)
		return;
		
	list_for_each_entry_reverse(entry, &host_desc->partition_list->list, list)
	{
		if ((entry->start * blksz) <= start)
		{
			if(write)
			{
				entry->bytes_written += bytes_xfered;
			}
			else
			{
				entry->bytes_read += bytes_xfered;
			}
			break;
		}
	}
	spin_unlock_irqrestore(&host_desc->logger_lock, logger_flags);
}

int transfer_logger_add(struct mmc_host *host)
{
	struct host_description* new_host_desc;
	
	// Exit if we already know this host
	if (find_host_description(host) != NULL)
	{
		return 0;
	}
	
	if(class_device_create_file(&host->class_dev, &class_device_attr_transfer_log))
	{
		printk(KERN_ERR "Failed to add write_log attribute.\n");
	}
	else
	{
		new_host_desc = kmalloc(sizeof(struct host_description), GFP_KERNEL);
		if (new_host_desc == NULL)
		{
			class_device_remove_file(&host->class_dev, &class_device_attr_transfer_log);
			return -ENODEV;
		}

		new_host_desc->host = host;

		new_host_desc->partition_list = 
		kmalloc(sizeof(struct partition_description), GFP_KERNEL);
		if (new_host_desc->partition_list == NULL)
		{
			class_device_remove_file(&host->class_dev, &class_device_attr_transfer_log);
			printk(KERN_ERR "Failed to allocate write_logger memory.\n");
			kfree(new_host_desc);
			return -ENODEV;
		}

		memset(new_host_desc->partition_list,0,sizeof(struct partition_description));
		INIT_LIST_HEAD(&new_host_desc->partition_list->list);
		spin_lock_init(&new_host_desc->logger_lock);
		list_add_tail(&new_host_desc->list, &mmc_host_list.list);
	}
	return 0;
}

int transfer_logger_remove(struct mmc_host *host)
{
	unsigned long logger_flags;
	struct host_description* host_desc;

	host_desc = find_host_description(host);
	if (host_desc == NULL)
	{
		return -ENODEV;
	}

	spin_lock_irqsave(&host_desc->logger_lock, logger_flags);
	if (host_desc->partition_list != NULL)
	{
		class_device_remove_file(&host->class_dev, &class_device_attr_transfer_log);
		clear_partition_list(host_desc);
		kfree(host_desc->partition_list);
	}
	spin_unlock_irqrestore(&host_desc->logger_lock, logger_flags);
	list_del(&host_desc->list);
	kfree(host_desc);
	return 0;
}