#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/gpio.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <asm/cacheflush.h>
#include <linux/semaphore.h>


MODULE_AUTHOR("wxc");
MODULE_LICENSE("GPL");


struct pci_dev *gDev = NULL;
unsigned int gBasePhys;
unsigned int gBaseLen;
char *gBaseVirt;
char *gDmaVirt;
unsigned int gDmaPhys;
#define DMA_BUFFER_SIZE_M 0x100000
#define DMA_BUFFER_SIZE   1*DMA_BUFFER_SIZE_M
enum {
	Bar0 = 0,
	Bar1 = 1,
	Bar2 = 2
};

#define FPGA_DMA

#ifdef FPGA_DMA
#define DMA_TRANSFER_SIZE 0x2000000
#define DMA_BASE_ADDR     0x20000000
#define NXP_PCIE0_CFG_BASE 0x98000000
#endif

#define BAR1
//#define BAR2

#ifdef BAR1
static unsigned int gBase1Phys;
static unsigned int gBase1Len;
static char *gBase1Virt;
#endif

#ifdef BAR2
static unsigned int gBase2Phys;
static unsigned int gBase2Len;
static char *gBase2Virt;
#endif

char gDevName[] = "dev_pcie_nxp";

unsigned int gStatFlags = 0x00;
#define HAVE_CDEV_REG     0x01
#define HAVE_REGION_BASE  0x02
#define HAVE_DMA_REGION   0x04
#define HAVE_DMA_CHANNEL  0x08
#define HAVE_REGION_BASE1 0x10
#define HAVE_REGION_BASE2 0x20

#define HAVE_IRQ		  0x20


#define MMAP_MINOR       0
#define MMAP_MINOR_COUNT 1
#define MMAP_DEV_COUNT   1 
#define MMAP_DRV_NAME    "mmap_nxp"

#ifndef FPGA_DMA
struct dma_chan *dma_channel;
struct dma_async_tx_descriptor *desc;
#endif

static struct mmap_dev {
	struct cdev dev;
	int    devno;
}mmap_dev;

static struct class *mmap_class;

enum {
	START_TRANSFER = 3,
	BAR_MEM_TEST,
    GET_GPIO_STATE,
	SET_GPIO_STATE,
	CLEAR_POLLMASK,
	SEND_SPI_DATA,
	FIRST_TIME_MMAP,
	IOCTL_TEST,
	HEAD_MSG
};

typedef struct {
	int pin;
	int state;
}gpio_st;

typedef struct {
	unsigned int virt;
	unsigned int phys;
	unsigned int len;
	void *next;
}vpl;

vpl *vpl_head, *vpl_tail;

typedef struct {
	unsigned int virt;
	unsigned int len;
}virtlen;


int gOpen = 0;

struct timeval gTv1, gTv2;

unsigned int pollmask;
DEFINE_SPINLOCK(pollmask_spinlock);
DEFINE_SEMAPHORE(dmatransfer_sema);
DECLARE_WAIT_QUEUE_HEAD(poll_q);


static int irq_trigger;


#define CPU_OUT_GX_READY_FOR_DATA 44
#define	CPU_IN_ONE_PASS_FINISHED  27
#define	CPU_IN_IS_DATA_ENABLE     46


#define CPU_OUT_SPI_CMD  49
#define CPU_OUT_SPI_DATA 48
#define CPU_OUT_SPI_CLK  57


#ifdef FPGA_DMA
void nxp_pci_dma_write_reg(unsigned int dw_offset, unsigned int val) {
	writel(val, gBaseVirt + 4 * dw_offset);
}
void nxp_pci_dma_read_reg(unsigned int dw_offset, unsigned int *reg_val) {
	*reg_val = readl(gBaseVirt + 4 * dw_offset);
}
#endif

void read_head_msg(char *buf, int len) {
	int i = 0;
	for (i = 0; i < len; i += 4) 
        nxp_pci_dma_read_reg(25 + i, (unsigned int *)(buf + i * 4));
}

unsigned int get_virt_to_phys(virtlen *vl) {
	vpl *tmp = vpl_head;
	while (tmp != NULL) {
		if (vl->virt >= tmp->virt &&
				vl->virt+vl->len <= tmp->virt+tmp->len)
			break;
		else
			tmp = (vpl *)(tmp->next);
	}
	if (tmp == NULL)
		return 0;
	else
	    return vl->virt - tmp->virt + tmp->phys;
}

int pcie_nxp_open(struct inode *inode, struct file *filp) {
	if (gOpen == 0) {
		gOpen = 1;
		return 0;
	}
	else {
		printk("[%s %d] pcie nxp device busy\n", __func__, __LINE__);
		return -EBUSY;
	}
}

int pcie_nxp_release(struct inode *inode, struct file *filp) {
	gOpen = 0;
	return 0;
}
#ifndef FPGA_DMA
static bool dma_chan_filter(struct dma_chan *chan, void *param) {
	return  1;
}
static int request_channel(void) {
	dma_cap_mask_t mask;
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
    dma_channel = dma_request_channel(mask, dma_chan_filter, NULL);
	return dma_channel == NULL;
}
static void dma_finish_callback(void *param) {
/*
    do_gettimeofday(&gTv2);
	printk("[%s %d] dma transfer time = %ldus\n", __func__, __LINE__,
			(gTv2.tv_sec-gTv1.tv_sec)*1000000+(gTv2.tv_usec-gTv1.tv_usec));
*/
	up(&dmatransfer_sema);
	
}
static int get_descriptor(unsigned int phys, int transfer_len) {
	dma_addr_t dest = gBasePhys;
	
	desc = dma_channel->device->device_prep_dma_memcpy(dma_channel, dest, phys,
			transfer_len, DMA_CTRL_ACK | DMA_COMPL_SKIP_SRC_UNMAP | DMA_COMPL_SKIP_DEST_UNMAP);

	desc->callback = dma_finish_callback;
	return 0;
}
#endif
void start_transfer(unsigned long arg) {
	virtlen *vl = (virtlen *)arg;
	unsigned int phys = get_virt_to_phys(vl), reg_val; 
	char msg[8], *vlv = NULL;
	int i = 0, len, delay_count = 0;
#ifndef FPGA_DMA
	dma_cookie_t cookie;
#endif

	//printk("[%s %d] transfer buffer : virt = 0x%x, phys = 0x%x, len = 0x%x\n", __func__, __LINE__,
	//		vl->virt, phys, vl->len);

	if (vl->len == 8) { //head msg
		vlv = (char *)(vl->virt);
		for (i = 0; i < 8; ++i) {
			msg[i] = vlv[7-i];
			printk("0x%x ", msg[i]);
		}
		printk("\n");
#ifdef BAR1
		memcpy(gBase1Virt, msg, 8);	
#endif
	}
	else if (phys == 0) {
		printk("[%s %d] start dma transfer, phys error\n", __func__, __LINE__);
	}
	else { //data
#ifdef FPGA_DMA
		printk("[%s %d] fpga dma transfer start, phys = 0x%x\n", __func__, __LINE__, phys);

		//2017-10-27
#if 0
		for (i = 0; i < vl->len; i += 1) {
			if (((char *)(vl->virt))[i] != 0x55) {
				printk("[%s %d] correct data error at %d\n", __func__, __LINE__, i);
				break;
			}
		}
		printk("[%s %d] data correct \n", __func__, __LINE__);
#endif
		do_gettimeofday(&gTv1);
		for (i = 0; i < 4 && vl->len > 0; ++i) {
			len = 0x800000;
			if (vl->len < len) 
				len = vl->len;
			vl->len -= len;

			if (i > 0) {
			    delay_count = 0;
				do {
					udelay(100);
					nxp_pci_dma_read_reg(0x01, &reg_val);
					if (++delay_count == 1000)
						break;
				} while (!(reg_val & (1 << 24)));
				if (delay_count == 1000)
					break;
			}

			nxp_pci_dma_write_reg(0x08, 0x20);
			nxp_pci_dma_write_reg(0x09, len/ 0x80);
            printk("[%s %d] delay_count = %d, tlp count = 0x%x\n", __func__, __LINE__, delay_count, len / 0x80);
			nxp_pci_dma_write_reg(0, 1);
			nxp_pci_dma_write_reg(0, 0);

			nxp_pci_dma_write_reg(0x07, phys + i * 0x800000);
			nxp_pci_dma_write_reg(0x01, 0x10000);
		}


#else
		printk("[%s %d] cpu dma transfer start\n", __func__, __LINE__);
		do_gettimeofday(&gTv1);
		down(&dmatransfer_sema);
		do_gettimeofday(&gTv2);
		printk("[%s %d] wait time = %ldus\n", __func__, __LINE__,
			(gTv2.tv_sec-gTv1.tv_sec)*1000000+(gTv2.tv_usec-gTv1.tv_usec));
		get_descriptor(phys, vl->len);
		get_descriptor(phys, vl->len);
		cookie = dmaengine_submit(desc);
		do_gettimeofday(&gTv1);
		dma_async_issue_pending(dma_channel);
#endif
	}
}

void clear_irq(void) {
	unsigned long flag;

	spin_lock_irqsave(&pollmask_spinlock, flag);
	pollmask = 0;
	spin_unlock_irqrestore(&pollmask_spinlock, flag);
}

void spi_send_bit(unsigned int bit) {
	if (bit) {
		gpio_set_value(CPU_OUT_SPI_DATA, 1);
//		printk("[%s %d] spi data bit 1\n", __func__, __LINE__);
	} else {
		gpio_set_value(CPU_OUT_SPI_DATA, 0);
//		printk("[%s %d] spi data bit 0\n", __func__, __LINE__);
	}
	udelay(1);
	gpio_set_value(CPU_OUT_SPI_CLK, 1);
	udelay(5);
	gpio_set_value(CPU_OUT_SPI_CLK, 0);
}

unsigned int reverse(unsigned int val) {
	unsigned int ll = val & 0x000000ff;
	unsigned int lh = val & 0x0000ff00;


	unsigned int hl = val & 0x00ff0000;
	unsigned int hh = val & 0xff000000;

	unsigned int res = (ll << 24) | (lh << 8) | (hl >> 8) | (hh >> 24);

	return res;
}



long pcie_nxp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	struct timeval tv1, tv2;
	long time = 0;
	char *p = (char *)arg;
	gpio_st *gs = (gpio_st *)arg;
	virtlen *vl = (virtlen *)arg;
	int len = 0, i, delay_count;
	unsigned int spi_data, reg_val;
	//char hmsg[4];

//    printk("[%s %d]\n", __func__, __LINE__);

	switch(cmd) {
		case START_TRANSFER:
#if 0
			read_head_msg(hmsg, 4);
			printk("[%s %d] read head msg %x %x %x %x\n", __func__,__LINE__, hmsg[0], hmsg[1], hmsg[2], hmsg[3]);
#endif
			flush_cache_all();
			start_transfer(arg);
			break;
		case BAR_MEM_TEST:
			do_gettimeofday(&tv1);
#ifdef BAR1
			memcpy(gBase1Virt, p, 0x08);
#endif
			memcpy(gBaseVirt, p, 0x2000000);
			//memcpy(gDmaVirt, p, 0x100000);
			do_gettimeofday(&tv2);
			time = (tv2.tv_sec-tv1.tv_sec)*1000000+(tv2.tv_usec-tv1.tv_usec);
			printk("[%s %d] bar memory test, write data %d, time = %ldus\n", __func__, __LINE__, p[0], 
					time);
			mdelay(10);
			break;
		case GET_GPIO_STATE:
			gs->state = gpio_get_value(gs->pin);
			break;
		case SET_GPIO_STATE:
			gpio_set_value(gs->pin, gs->state);
			break;
		case CLEAR_POLLMASK:
			clear_irq();
			break;
		case SEND_SPI_DATA:
			spi_data = vl->virt;
			len = vl->len;
	//		printk("[%s %d] spi_data 0x%x, len %d\n", __func__, __LINE__, spi_data, len);
			while (len > 0) {
				spi_send_bit(spi_data & (unsigned int)0x80000000);
				spi_data <<= 1;
				--len;
			}
	//		printk("[%s %d] spi send data finished\n", __func__, __LINE__);
			gpio_set_value(CPU_OUT_SPI_CMD, 1); 
			udelay(5);
			gpio_set_value(CPU_OUT_SPI_CMD, 0); 
			gpio_set_value(CPU_OUT_SPI_DATA, 0); 
			gpio_set_value(CPU_OUT_SPI_CLK, 0); 
			break;
		case FIRST_TIME_MMAP:
            while (vpl_head) {
				vpl_tail = (vpl *)(vpl_head->next);
                kfree(vpl_head);
				vpl_head = vpl_tail;
			}
			break;
		case IOCTL_TEST:
#ifdef FPGA_DMA
			memcpy(gBase1Virt, p, 0x08);
			printk("[%s %d] start fpga dma transfer\n", __func__, __LINE__);
		    do_gettimeofday(&gTv1);
			nxp_pci_dma_write_reg(0x08, 0x20);
			nxp_pci_dma_write_reg(0x09, 0x800000 / 0x80);

			for (i = 0; i < 4; ++i) {

				if (i > 0) {
					delay_count = 0;
					do {
						udelay(100);
						nxp_pci_dma_read_reg(0x01, &reg_val);
						if (++delay_count == 1000)
							break;
					} while (!(reg_val & (1 << 24)));
					if (delay_count == 1000)
						break;
				}

				nxp_pci_dma_write_reg(0, 1);
				nxp_pci_dma_write_reg(0, 0);
				nxp_pci_dma_write_reg(0x07, DMA_BASE_ADDR + i * 0x800000);
				nxp_pci_dma_write_reg(0x01, 0x10000);
			}

			mdelay(500);
			nxp_pci_dma_read_reg(0x01, &reg_val);
			printk("[%s %d] reg 0x01, val 0x%x\n", __func__, __LINE__, reg_val);

#endif
			break;
		case HEAD_MSG:
			read_head_msg((char *)(vl->virt), vl->len);
			break;
        default:
			break;
	}

    return 0;
}


#define RESERVED_BASE_PHYS 0x20000000
int pcie_nxp_dma_mmap(struct file *filp, struct vm_area_struct *vma) {
	long offset = vma->vm_pgoff << PAGE_SHIFT;
	vpl *p = NULL;

	if (remap_pfn_range(vma, vma->vm_start, (RESERVED_BASE_PHYS + offset) >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		printk("[%s %d] remap_pfn_range failed\n", __func__, __LINE__);
		return -1;
	}

    p = kmalloc(sizeof(*p), GFP_KERNEL); 
	if (p == NULL) {
		printk("[%s %d] vpl kmalloc failed\n", __func__, __LINE__);
	}
	else {
		p->virt = vma->vm_start;
		p->phys = RESERVED_BASE_PHYS + offset;
		p->len  = vma->vm_end - vma->vm_start;
		p->next = NULL;
		if (vpl_head == NULL)
			vpl_head = p;
		else 
			vpl_tail->next = p;
		vpl_tail = p;
//		printk("[%s %d] mmaped buffer : virt = 0x%x, phys = 0x%x, len = 0x%x\n", __func__, __LINE__,
//				p->virt, p->phys, p->len);
	}

	return 0;
}

static irqreturn_t nxp_irq_handler(int irq, void *dataarg) {

#ifdef FPGA_DMA
    do_gettimeofday(&gTv2);
	printk("[%s %d] fpga dma transfer time = %ldus\n", __func__, __LINE__,
			(gTv2.tv_sec-gTv1.tv_sec)*1000000+(gTv2.tv_usec-gTv1.tv_usec));
#endif

	spin_lock(&pollmask_spinlock);
	pollmask |= 1;
	spin_unlock(&pollmask_spinlock);
	wake_up_interruptible(&poll_q);

    return IRQ_HANDLED;
}

static int nxp_irq_init(void) {
    int ret = 0;

	gpio_request(CPU_OUT_GX_READY_FOR_DATA, "CPU_OUT_GX_READY_FOR_DATA");
	gpio_direction_output(CPU_OUT_GX_READY_FOR_DATA, 0);
	gpio_request(CPU_IN_ONE_PASS_FINISHED, "CPU_IN_ONE_PASS_FINISHED");
	gpio_direction_input(CPU_IN_ONE_PASS_FINISHED);
	gpio_request(CPU_IN_IS_DATA_ENABLE, "CPU_IN_IS_DATA_ENABLE");
	gpio_direction_input(CPU_IN_IS_DATA_ENABLE);


	gpio_request(CPU_OUT_SPI_CLK, "CPU_OUT_SPI_CLK");
	gpio_direction_output(CPU_OUT_SPI_CLK, 0);
	gpio_request(CPU_OUT_SPI_DATA, "CPU_OUT_SPI_DATA");
	gpio_direction_output(CPU_OUT_SPI_DATA, 0);
	gpio_request(CPU_OUT_SPI_CMD, "CPU_OUT_SPI_CMD");
	gpio_direction_output(CPU_OUT_SPI_CMD, 0);

	//gpio_request(gpio, "CPU_IN_FPGA_SEND_DATA");
	//gpio_direction_input(gpio);
	//irq_trigger = gpio_to_irq(gpio);
	irq_trigger = 83;
	if ((ret = request_irq(irq_trigger, nxp_irq_handler, IRQF_TRIGGER_RISING, "FPGA_SEND_DATA_IRQ", NULL)) != 0) {
		printk("[%s %d] request fpga_send_data_irq failed, ret = %d\n", __func__, __LINE__, ret);
		return -1;            
	}   
	printk("[%s %d] request fpga_send_data_irq succeed\n", __func__, __LINE__);
    gStatFlags |= HAVE_IRQ;

	return 0;
}

static unsigned int pcie_nxp_poll(struct file *filp, struct poll_table_struct *wait) {
	unsigned int mask;
	unsigned long flag;

	poll_wait(filp, &poll_q, wait);
	spin_lock_irqsave(&pollmask_spinlock, flag);
	mask = pollmask;
	spin_unlock_irqrestore(&pollmask_spinlock, flag);

	return mask;
}

static struct file_operations pcie_nxp_ops = {
    unlocked_ioctl:		pcie_nxp_ioctl,
	open:				pcie_nxp_open,
	release:			pcie_nxp_release,
	mmap:				pcie_nxp_dma_mmap,
	poll:				pcie_nxp_poll,
};

static int nxp_irq_destroy(void) {
	free_irq(irq_trigger, NULL);

    gpio_free(CPU_OUT_SPI_CMD); 	
    gpio_free(CPU_OUT_SPI_DATA);
    gpio_free(CPU_OUT_SPI_CLK); 

    gpio_free(CPU_OUT_GX_READY_FOR_DATA);	
    gpio_free(CPU_IN_ONE_PASS_FINISHED); 
    gpio_free(CPU_IN_IS_DATA_ENABLE);    

	return 0;
}

static int pcie_nxp_probe(struct pci_dev *pdev, const struct pci_device_id *ent) {
	int ret = 0;
	char *p;
    gDev = pdev;

	
	//enable pcie device 
	if (pci_enable_device(pdev) < 0) {
        printk("[%s %d] pcie device enable failed\n", __func__, __LINE__);
		return 0;
	}

	//Bar0
	gBasePhys = pci_resource_start(pdev, Bar0);
	if (gBasePhys < 0) {
		printk("[%s %d] bar %d: phys addr is not set\n", __func__, __LINE__, Bar0);
		return 0;
	}
	gBaseLen = pci_resource_len(pdev, Bar0);
	printk("[%s %d] phys addr = 0x%x, phys len = 0x%x\n", __func__, __LINE__,
			gBasePhys, gBaseLen);

	if (check_mem_region(gBasePhys, gBaseLen) < 0) {
		printk("[%s %d] Memory 0x%x, 0x%x in use\n", __func__, __LINE__,
				gBasePhys, gBaseLen);
		return 0;
	}
    request_mem_region(gBasePhys, gBaseLen, gDevName);
	gStatFlags |= HAVE_REGION_BASE;

	gBaseVirt = ioremap(gBasePhys, gBaseLen);
	if (gBaseVirt == NULL) {
		printk("[%s %d] could not remap memory in 0x%x\n", __func__, __LINE__, gBasePhys);
		goto err_ioremap;
	}
	else
		printk("[%s %d] BaseVirt = 0x%p\n", __func__, __LINE__, gBaseVirt);


	//Bar1
#ifdef BAR1
	gBase1Phys = pci_resource_start(pdev, Bar1);
	if (gBase1Phys < 0) {
		printk("[%s %d] bar %d: phys addr is not set\n", __func__, __LINE__, Bar1);
		goto err_bar1;
	}
	gBase1Len = pci_resource_len(pdev, Bar1);
	printk("[%s %d] phys addr = 0x%x, phys len = 0x%x\n", __func__, __LINE__,
			gBase1Phys, gBase1Len);

	if (check_mem_region(gBase1Phys, gBase1Len) < 0) {
		printk("[%s %d] Memory 0x%x, 0x%x in use\n", __func__, __LINE__,
				gBase1Phys, gBase1Len);
		goto err_bar1;
	}
    request_mem_region(gBase1Phys, gBase1Len, gDevName);
	gStatFlags |= HAVE_REGION_BASE1;

	gBase1Virt = ioremap(gBase1Phys, gBase1Len);
	if (gBase1Virt == NULL) {
		printk("[%s %d] could not remap memory in 0x%x\n", __func__, __LINE__, gBase1Phys);
		goto err_bar1_ioremap;
	}
	else
		printk("[%s %d] Base1Virt = 0x%p\n", __func__, __LINE__, gBase1Virt);
#endif

#ifdef BAR2
	gBase2Phys = pci_resource_start(pdev, Bar2);
	if (gBase2Phys < 0) {
		printk("[%s %d] bar %d: phys addr is not set\n", __func__, __LINE__, Bar2);
		goto err_bar2;
	}
	gBase2Len = pci_resource_len(pdev, Bar2);
	printk("[%s %d] phys addr = 0x%x, phys len = 0x%x\n", __func__, __LINE__,
			gBase2Phys, gBase2Len);
	if (check_mem_region(gBase2Phys, gBase2Len) < 0) {
		printk("[%s %d] Memory 0x%x, 0x%x in use\n", __func__, __LINE__,
				gBase2Phys, gBase2Len);
		goto err_bar2;
	}
    request_mem_region(gBase2Phys, gBase2Len, gDevName);
	gStatFlags |= HAVE_REGION_BASE2;
	gBase2Virt = ioremap(gBase2Phys, gBase2Len);
	if (gBase2Virt == NULL) {
		printk("[%s %d] could not remap memory in 0x%x\n", __func__, __LINE__, gBase2Phys);
		goto err_bar2_ioremap;
	}
	else
		printk("[%s %d] Base2Virt = 0x%p\n", __func__, __LINE__, gBase2Virt);
#endif

    //dma phys
    gDmaVirt = pci_alloc_consistent(pdev, DMA_BUFFER_SIZE, &gDmaPhys);	
	if (gDmaVirt == NULL) {
		printk("[%s %d] unable to alloc dma buffer\n", __func__, __LINE__);
		goto err_dma_alloc;
	}
	printk("[%s %d] dma phys addr = 0x%x, len = 0x%x\n, virt = 0x%p", __func__, __LINE__,
			gDmaPhys, DMA_BUFFER_SIZE, gDmaVirt);
	gStatFlags |= HAVE_DMA_REGION;
  
#ifndef FPGA_DMA
	//dma channel
    ret = request_channel();
	if (ret) {
		printk("[%s %d] request channel failed\n", __func__, __LINE__);
	    goto err_dma_chan;	
	}
	gStatFlags |= HAVE_DMA_CHANNEL;
#endif

	ret = alloc_chrdev_region(&mmap_dev.devno, MMAP_MINOR, MMAP_MINOR_COUNT, MMAP_DRV_NAME);
	if (ret < 0) {
		printk("[%s %d] alloc chrdev region failed\n", __func__, __LINE__);
		goto err_cdev_reg;
	}

	cdev_init(&mmap_dev.dev, &pcie_nxp_ops);
	mmap_dev.dev.owner = THIS_MODULE;

	ret = cdev_add(&mmap_dev.dev, mmap_dev.devno, MMAP_DEV_COUNT);
	if (ret < 0) {
		printk("[%s %d] cdev add failed\n", __func__, __LINE__);
		goto err_reg_region;
	}

	printk("[%s %d] register chrdev mmap_nxp succeed\n", __func__, __LINE__);

	mmap_class = class_create(THIS_MODULE, MMAP_DRV_NAME);
	if (IS_ERR(mmap_class)) {
		printk("[%s %d] class create failed\n", __func__, __LINE__);
		ret = PTR_ERR(mmap_class);
		goto err_cdev;
	}
	device_create(mmap_class, NULL, MKDEV(MAJOR(mmap_dev.devno), 0), NULL, MMAP_DRV_NAME); 
	gStatFlags |= HAVE_CDEV_REG;

	printk("[%s %d] load driver succeed\n", __func__, __LINE__);

    nxp_irq_init(); //TODO

    //max_payload_size setting
	p = (void __iomem *) (0xf5000000);
	if (p != NULL) {
		writel((readl(p + 0x70 + 0x08) & (~0xe0)) | 0x00, p + 0x70 + 0x08);
		printk("[%s %d] NXP_PCIE0_CFG_BASE, 0x%x\n", __func__, __LINE__, readl(p + 0x70 + 0x08));
	}

	return 0;
err_cdev:
	cdev_del(&mmap_dev.dev);
err_reg_region:
    unregister_chrdev_region(mmap_dev.devno, MMAP_DEV_COUNT);
err_cdev_reg:
#ifndef FPGA_DMA
	dma_release_channel(dma_channel);
err_dma_chan:
#endif
	pci_free_consistent(gDev, DMA_BUFFER_SIZE, gDmaVirt, gDmaPhys);
err_dma_alloc:

#ifdef BAR2
	iounmap(gBase2Virt);
err_bar2_ioremap:
	release_mem_region(gBase2Phys, gBase2Len);
err_bar2:
#endif

#ifdef BAR1
	iounmap(gBase1Virt);
err_bar1_ioremap:
	release_mem_region(gBase1Phys, gBase1Len);
err_bar1:
#endif
	iounmap(gBaseVirt);
err_ioremap:
	release_mem_region(gBasePhys, gBaseLen);

	return 0;
}

static void pcie_nxp_remove(struct pci_dev *pdev) {
    if (gStatFlags & HAVE_CDEV_REG) {
		device_destroy(mmap_class, MKDEV(MAJOR(mmap_dev.devno), 0));
		class_destroy(mmap_class);
		cdev_del(&mmap_dev.dev);
		unregister_chrdev_region(mmap_dev.devno, MMAP_DEV_COUNT);
	}
	if (gBaseVirt != NULL)
		iounmap(gBaseVirt);
#ifdef BAR1
	if (gBase1Virt != NULL)
		iounmap(gBase1Virt);
#endif
	if (gStatFlags & HAVE_REGION_BASE)
		release_mem_region(gBasePhys, gBaseLen);
#ifdef BAR1
	if (gStatFlags & HAVE_REGION_BASE1)
		release_mem_region(gBase1Phys, gBase1Len);
#endif

#ifdef BAR2
	if (gBase2Virt != NULL)
		iounmap(gBase2Virt);
	if (gStatFlags & HAVE_REGION_BASE2)
		release_mem_region(gBase2Phys, gBase2Len);
#endif

	if (gStatFlags & HAVE_DMA_REGION)
		pci_free_consistent(gDev, DMA_BUFFER_SIZE, gDmaVirt, gDmaPhys);
#ifndef FPGA_DMA
	if (gStatFlags & HAVE_DMA_CHANNEL)
		dma_release_channel(dma_channel);
#endif
	if (gStatFlags & HAVE_IRQ)
		nxp_irq_destroy();

	while (vpl_head) {
		vpl_tail = (vpl *)(vpl_head->next);
		kfree(vpl_head);
		vpl_head = vpl_tail;
	}

	printk("[%s %d] remove driver succeed\n", __func__, __LINE__);
}

static DEFINE_PCI_DEVICE_TABLE(pcie_nxp_tbl) = {
	{PCI_DEVICE(0x10EE , 0x7021), 0, 0, 0},
	{0,},
};

static struct pci_driver pcie_nxp_driver = {
	.name		= "pcie_nxp",
	.id_table	= pcie_nxp_tbl,
	.probe		= pcie_nxp_probe,
	.remove		= pcie_nxp_remove,
};

static void __exit pcie_nxp_module_exit(void) {
	pci_unregister_driver(&pcie_nxp_driver);
}
static int __init pcie_nxp_module_init(void) {
	return pci_register_driver(&pcie_nxp_driver);
}

module_init(pcie_nxp_module_init);
module_exit(pcie_nxp_module_exit);
