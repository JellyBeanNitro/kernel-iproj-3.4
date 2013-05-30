#platform.team@lge.com	2011.01

ifeq ($(CONFIG_MSM_CAMERA_V4L2),y)
  GCC_VERSION      := $(shell $(CONFIG_SHELL) $(PWD)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)
  EXTRA_CFLAGS += -Idrivers/media/video/msm
  EXTRA_CFLAGS += -Idrivers/media/video/msm/io
  EXTRA_CFLAGS += -Idrivers/media/video/msm/eeprom
  EXTRA_CFLAGS += -Idrivers/media/video/msm/csi
  EXTRA_CFLAGS += -Idrivers/media/video/msm/sensors
  EXTRA_CFLAGS += -Idrivers/media/video/msm/actuators  
endif
  
ifeq ($(CONFIG_MSM_CAMERA_V4L2),y)
  obj-$(CONFIG_LGE_SENSOR_MT9E013)+= sensor_mt9e013_v4l2.o
  obj-$(CONFIG_LGE_SENSOR_MT9P017)+= sensor_mt9p017_v4l2.o  
  obj-$(CONFIG_LGE_SENSOR_IMX105)+= sensor_imx105_v4l2.o  
  obj-$(CONFIG_LGE_SENSOR_MT9M114)+= sensor_mt9m114_v4l2.o  
else
  obj-$(CONFIG_LGE_SENSOR_MT9E013)+= sensor_mt9e013.o sensor_mt9e013_reg.o
  obj-$(CONFIG_LGE_SENSOR_IMX105)+= sensor_imx105.o sensor_imx105_reg.o
  obj-$(CONFIG_LGE_SENSOR_ISX005)+= sensor_isx005.o sensor_isx005_reg.o sensor_isx005_tun.o
  obj-$(CONFIG_LGE_SENSOR_ISX006)+= sensor_isx006.o sensor_isx006_reg.o sensor_isx006_tun.o
  obj-$(CONFIG_LGE_SENSOR_MT9M113)+= sensor_mt9m113.o sensor_mt9m113_reg.o  
  obj-$(CONFIG_LGE_SENSOR_MT9M114)+= sensor_mt9m114.o sensor_mt9m114_reg.o  
endif

obj-$(CONFIG_LGE_FLASH_AAT1270)+= flash_aat1270.o
obj-$(CONFIG_LGE_FLASH_LM3559)+= flash_lm3559.o lge_flash.o
obj-$(CONFIG_LGE_PM_BH6172)+= pm_bh6172.o
