
#include <config.h>
#include <common.h>

#include <asm/arch/pmic.h>

/* MIPI */
#include <asm/arch/dsim.h>

/* MDNIE PWM */
#include <asm/arch/cpu.h>
#include "s5p_mdnie.h"

#define ID_7IN_BOE	0xB8
#define ID_7IN_SDC	0x88

#define LDI_ID_REG		0xD8
#define LDI_ID_LEN		3

/* S6D7AA0 */
static const unsigned char SEQ_PASSWD1[] = {
	0xF0,
	0x5A, 0x5A
};

static const unsigned char SEQ_PASSWD2[] = {
	0xF1,
	0x5A, 0x5A
};

static const unsigned char SEQ_PASSWD3[] = {
	0xFC,
	0xA5, 0xA5
};

static const unsigned char SEQ_PASSWD1_LOCK[] = {
	0xF0,
	0xA5, 0xA5
};

static const unsigned char SEQ_PASSWD2_LOCK[] = {
	0xF1,
	0xA5, 0xA5
};

static const unsigned char SEQ_PASSWD3_LOCK[] = {
	0xFC,
	0x5A, 0x5A
};

static const unsigned char SEQ_OTP_RELOAD[] = {
	0xD0,
	0x00, 0x10
};

static const unsigned char SEQ_BC_PARAM_MDNIE[] = {
	0xBC,
	0x01, 0x4E, 0x0A
};

static const unsigned char SEQ_FD_PARAM_MDNIE[] = {
	0xFD,
	0x16, 0x10, 0x11, 0x23, 0x09
};

static const unsigned char SEQ_FE_PARAM_MDNIE[] = {
	0xFE,
	0x00, 0x02, 0x03, 0x21, 0x00, 0x78
};

static const unsigned char SEQ_B3_PARAM[] = {
	0xB3,
	0x51, 0x00
};

static unsigned char SEQ_BACKLIGHT_CTL[] = {
	0x53,
	0x2C, 0x00
};

static const unsigned char SEQ_PORCH_CTL[] = {
	0xF2,
	0x02, 0x08, 0x08
};

static unsigned char SEQ_BRIGHTNESS_DEFAULT[] = {
	0x51,
	0x51, 0x00
};

static unsigned char SEQ_BRIGHTNESS_UPLOAD[] = {
	0x51,
	0x14, 0x00
};

static const unsigned char SEQ_SLEEP_OUT[] = {
	0x11,
	0x00, 0x00
};

static const unsigned char SEQ_BL_ON_CTL[] = {
	0xC3,
	0x3B, 0x00, 0x2A
};

static const unsigned char SEQ_TEON_CTL[] = {
	0x35,
	0x00, 0x00
};

static const unsigned char SEQ_DISPLAY_ON[] = {
	0x29,
	0x00, 0x00
};

static void dsim_write(const unsigned char *wbuf, int size)
{
	if (!g_bd.lcd_connected)
		return;

	if (size == 1)
		s5p_dsim_wr_data(DSIM_BASE, DCS_WR_NO_PARA, wbuf[0], 0);
	else if (size == 2)
		s5p_dsim_wr_data(DSIM_BASE, DCS_WR_1_PARA, wbuf[0], wbuf[1]);
	else
		s5p_dsim_wr_data(DSIM_BASE, DCS_LONG_WR, (unsigned int)wbuf, size);
}

static int dsim_read(const u8 addr, u16 count, u8 *buf)
{
	int ret = 0;

	ret = s5p_dsim_rd_data(DSIM_BASE, addr, count, buf);

	return ret;
}

static int s6d7aa0_read_id(u8 *buf, u8 retry_cnt)
{
	int i;
	int ret;

retry:
	ret = dsim_read(LDI_ID_REG, LDI_ID_LEN, buf);

	if (!ret) {
		if (retry_cnt) {
			printf("%s :: retry: %d\n", __func__, retry_cnt);
			retry_cnt--;
			goto retry;
		} else {
			printf("%s :: 0x%02x\n", __func__, LDI_ID_REG);
			printf("panel is not connected well\n");
			g_bd.lcd_connected = 0;
		return 0;
		}
	}

	for (i = 0; i < LDI_ID_LEN; i++)
		printf("%x, ", buf[i]);
	printf("\n");

	return ret;
}

void lcd_power_on(void)
{
	pmic_ldo_disable(PMIC_LDO30);	/* LCD 1.8v */
	pmic_ldo_disable(PMIC_LDO29);	/* LCD 3.0v */
	mdelay(5);
	pmic_ldo_enable(PMIC_LDO30);	/* LCD 1.8v */
	pmic_ldo_enable(PMIC_LDO29);	/* LCD 3.0v */
}

void reset_lcd(void)
{
	s5p_gpio_setpin(GPIO_MLCD_RST, GPIO_LEVEL_HIGH);
	udelay(200);
	s5p_gpio_setpin(GPIO_MLCD_RST, GPIO_LEVEL_LOW);
	udelay(200);
	s5p_gpio_setpin(GPIO_MLCD_RST, GPIO_LEVEL_HIGH);
	mdelay(5);
}

void lcd_otp_init(void)
{
	dsim_write(SEQ_PASSWD1, ARRAY_SIZE(SEQ_PASSWD1));
	dsim_write(SEQ_PASSWD2, ARRAY_SIZE(SEQ_PASSWD2));
	dsim_write(SEQ_PASSWD3, ARRAY_SIZE(SEQ_PASSWD3));
	dsim_write(SEQ_OTP_RELOAD, ARRAY_SIZE(SEQ_OTP_RELOAD));
	msleep(5);
}

void lcd_id88_init(void)
{
	dsim_write(SEQ_BL_ON_CTL, ARRAY_SIZE(SEQ_BL_ON_CTL));
	dsim_write(SEQ_BC_PARAM_MDNIE, ARRAY_SIZE(SEQ_BC_PARAM_MDNIE));
	dsim_write(SEQ_FD_PARAM_MDNIE, ARRAY_SIZE(SEQ_FD_PARAM_MDNIE));
	dsim_write(SEQ_FE_PARAM_MDNIE, ARRAY_SIZE(SEQ_FE_PARAM_MDNIE));
	dsim_write(SEQ_B3_PARAM, ARRAY_SIZE(SEQ_B3_PARAM));
	dsim_write(SEQ_BACKLIGHT_CTL, ARRAY_SIZE(SEQ_BACKLIGHT_CTL));
	if (g_bd.sys_bootm & SYS_BOOTM_UP) {
		dsim_write(SEQ_BRIGHTNESS_UPLOAD, ARRAY_SIZE(SEQ_BRIGHTNESS_UPLOAD));
	} else {
		dsim_write(SEQ_BRIGHTNESS_DEFAULT, ARRAY_SIZE(SEQ_BRIGHTNESS_DEFAULT));
	}
	dsim_write(SEQ_PORCH_CTL, ARRAY_SIZE(SEQ_PORCH_CTL));
	msleep(10);
	dsim_write(SEQ_SLEEP_OUT, ARRAY_SIZE(SEQ_SLEEP_OUT));
	msleep(50);
 	dsim_write(SEQ_PASSWD1_LOCK, ARRAY_SIZE(SEQ_PASSWD1_LOCK));
	dsim_write(SEQ_PASSWD2_LOCK, ARRAY_SIZE(SEQ_PASSWD2_LOCK));
	dsim_write(SEQ_PASSWD3_LOCK, ARRAY_SIZE(SEQ_PASSWD3_LOCK));
	dsim_write(SEQ_TEON_CTL, ARRAY_SIZE(SEQ_TEON_CTL));
	dsim_write(SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));
}

void lcd_idB8_init(void)
{
	dsim_write(SEQ_BL_ON_CTL, ARRAY_SIZE(SEQ_BL_ON_CTL));
	dsim_write(SEQ_BC_PARAM_MDNIE, ARRAY_SIZE(SEQ_BC_PARAM_MDNIE));
	dsim_write(SEQ_FD_PARAM_MDNIE, ARRAY_SIZE(SEQ_FD_PARAM_MDNIE));
	dsim_write(SEQ_FE_PARAM_MDNIE, ARRAY_SIZE(SEQ_FE_PARAM_MDNIE));
	dsim_write(SEQ_B3_PARAM, ARRAY_SIZE(SEQ_B3_PARAM));
	dsim_write(SEQ_BACKLIGHT_CTL, ARRAY_SIZE(SEQ_BACKLIGHT_CTL));
	if (g_bd.sys_bootm & SYS_BOOTM_UP) {
		dsim_write(SEQ_BRIGHTNESS_UPLOAD, ARRAY_SIZE(SEQ_BRIGHTNESS_UPLOAD));
	} else {
		dsim_write(SEQ_BRIGHTNESS_DEFAULT, ARRAY_SIZE(SEQ_BRIGHTNESS_DEFAULT));
	}
	dsim_write(SEQ_PORCH_CTL, ARRAY_SIZE(SEQ_PORCH_CTL));
	msleep(10);
	dsim_write(SEQ_SLEEP_OUT, ARRAY_SIZE(SEQ_SLEEP_OUT));
	msleep(50);
	dsim_write(SEQ_PASSWD1_LOCK, ARRAY_SIZE(SEQ_PASSWD1_LOCK));
	dsim_write(SEQ_PASSWD2_LOCK, ARRAY_SIZE(SEQ_PASSWD2_LOCK));
	dsim_write(SEQ_PASSWD3_LOCK, ARRAY_SIZE(SEQ_PASSWD3_LOCK));
	dsim_write(SEQ_TEON_CTL, ARRAY_SIZE(SEQ_TEON_CTL));
	dsim_write(SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));
}

void lcd_panel_init(void)
{
	unsigned char id_buf[LDI_ID_LEN] = {0,};
	g_bd.lcd_connected = 1;
	lcd_otp_init();
	s6d7aa0_read_id(id_buf, 3);

	if (id_buf[1] == ID_7IN_BOE)
		lcd_idB8_init();
	else if (id_buf[1]  == ID_7IN_SDC)
		lcd_id88_init();
	else
		lcd_idB8_init();
}

int lcd_start(void)
{
	int ret = 0;
	printf("+%s\n", __func__);

	s5p_dsim_init();

	printf("-%s\n", __func__);
	return ret;
}
