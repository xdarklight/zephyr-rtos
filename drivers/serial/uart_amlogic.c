/*
 * Copyright (c) 2021 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT			amlogic_meson_uart

#include <device.h>
#include <drivers/uart.h>
#include <sys/sys_io.h>

/* Register offsets */
#define AML_UART_WFIFO			0x00
#define AML_UART_RFIFO			0x04
#define AML_UART_CONTROL		0x08
#define AML_UART_STATUS			0x0c
#define AML_UART_MISC			0x10
#define AML_UART_REG5			0x14

/* AML_UART_CONTROL bits */
#define AML_UART_TX_EN			BIT(12)
#define AML_UART_RX_EN			BIT(13)
#define AML_UART_TWO_WIRE_EN		BIT(15)
#define AML_UART_STOP_BIT_LEN_MASK	GENMASK(17, 16)
#define AML_UART_STOP_BIT_1SB		(0x00 << 16)
#define AML_UART_STOP_BIT_2SB		(0x01 << 16)
#define AML_UART_PARITY_TYPE		BIT(18)
#define AML_UART_PARITY_EN		BIT(19)
#define AML_UART_TX_RST			BIT(22)
#define AML_UART_RX_RST			BIT(23)
#define AML_UART_CLEAR_ERR		BIT(24)
#define AML_UART_RX_INT_EN		BIT(27)
#define AML_UART_TX_INT_EN		BIT(28)
#define AML_UART_DATA_LEN_MASK		GENMASK(21, 20)
#define AML_UART_DATA_LEN_8BIT		(0x00 << 20)
#define AML_UART_DATA_LEN_7BIT		(0x01 << 20)
#define AML_UART_DATA_LEN_6BIT		(0x02 << 20)
#define AML_UART_DATA_LEN_5BIT		(0x03 << 20)

/* AML_UART_STATUS bits */
#define AML_UART_RX_FIFO_BYTES		GENMASK(7, 0)
#define AML_UART_TX_FIFO_BYTES		GENMASK(14, 8)
#define AML_UART_PARITY_ERR		BIT(16)
#define AML_UART_FRAME_ERR		BIT(17)
#define AML_UART_TX_FIFO_WERR		BIT(18)
#define AML_UART_RX_EMPTY		BIT(20)
#define AML_UART_TX_FULL		BIT(21)
#define AML_UART_TX_EMPTY		BIT(22)
#define AML_UART_XMIT_BUSY		BIT(25)

#define AML_MAX_FIFO_BYTES		64

struct uart_amlogic_data {
	const struct device *clock;
	struct uart_config ucfg;
};

static void meson_amlogic_wait_tx_fifo_empty(const struct device *dev)
{
	const struct uart_device_config *config = dev->config;
	uint32_t status;

	/*
	 * NOTE: it's unclear why AML_UART_TX_FULL and AML_UART_TX_EMPTY always
	 * reads as 0.
	 */
	do {
		status = sys_read32(config->regs + AML_UART_STATUS);
	} while ((status & AML_UART_TX_FIFO_BYTES) != 0);
}

static void meson_amlogic_put_char(const struct device *dev, uint32_t c)
{
	const struct uart_device_config *config = dev->config;

	meson_amlogic_wait_tx_fifo_empty(dev);

	sys_write32(c, config->regs + AML_UART_WFIFO);
}

static int uart_amlogic_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_device_config *config = dev->config;
	uint32_t status;

	status = sys_read32(config->regs + AML_UART_STATUS);
	if ((status & AML_UART_RX_FIFO_BYTES) == 0)
		return -1;

	*c = sys_read32(config->regs + AML_UART_RFIFO) & BIT_MASK(8);
	return 0;
}

static void uart_amlogic_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_device_config *config = dev->config;
	uint32_t control;

	control = sys_read32(config->regs + AML_UART_CONTROL);

	/* temporarily switch to polled mode */
	sys_write32(control & ~(AML_UART_TX_INT_EN | AML_UART_RX_INT_EN),
		    config->regs + AML_UART_CONTROL);

	meson_amlogic_put_char(dev, c);

	/* restore the original control settings */
	sys_write32(control, config->regs + AML_UART_CONTROL);
}

static int uart_amlogic_err_check(const struct device *dev)
{
	const struct uart_device_config *config = dev->config;
	uint32_t status, control;
	int errors = 0;

	status = sys_read32(config->regs + AML_UART_STATUS);

	if (status & AML_UART_TX_FIFO_WERR)
		errors |= UART_ERROR_OVERRUN;

	if (status & AML_UART_PARITY_ERR)
		errors |= UART_ERROR_PARITY;

	if (status & AML_UART_FRAME_ERR)
		errors |= UART_ERROR_FRAMING;

	if (errors) {
		/* clear any pending errors */
		control = sys_read32(config->regs + AML_UART_CONTROL);
		sys_write32(control | AML_UART_CLEAR_ERR,
			    config->regs + AML_UART_CONTROL);
		sys_write32(control, config->regs + AML_UART_CONTROL);
	}

	return errors;
}

static const struct uart_driver_api uart_amlogic_driver_api = {
	.poll_in	= uart_amlogic_poll_in,
	.poll_out	= uart_amlogic_poll_out,
	.err_check	= uart_amlogic_err_check,
};

static int uart_amlogic_init(const struct device *dev)
{
	const struct uart_device_config *config = dev->config;
	uint32_t control;

	control = sys_read32(config->regs + AML_UART_CONTROL);

	control |= (AML_UART_RX_EN | AML_UART_TX_EN);
	sys_write32(control, config->regs + AML_UART_CONTROL);

	meson_amlogic_wait_tx_fifo_empty(dev);

	control |= (AML_UART_RX_RST | AML_UART_TX_RST | AML_UART_CLEAR_ERR);
	sys_write32(control, config->regs + AML_UART_CONTROL);

	control &= ~(AML_UART_RX_RST | AML_UART_TX_RST | AML_UART_CLEAR_ERR);
	sys_write32(control, config->regs + AML_UART_CONTROL);

	return 0;
}

#define AMLOGIC_INIT(index)						\
									\
static const struct uart_device_config uart_amlogic_cfg_##index = {	\
	.base = (uint8_t *)DT_INST_REG_ADDR(index),			\
};									\
									\
static struct uart_amlogic_data uart_amlogic_data_##index = {		\
	.ucfg = {							\
		.baudrate = DT_INST_PROP(index, current_speed),		\
	},								\
};									\
									\
DEVICE_DT_INST_DEFINE(index,						\
		    &uart_amlogic_init,					\
		    device_pm_control_nop,				\
		    &uart_amlogic_data_##index,				\
		    &uart_amlogic_cfg_##index,				\
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
		    &uart_amlogic_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMLOGIC_INIT)
