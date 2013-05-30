ifdef CONFIG_ARCH_MSM8X60
#by mo2chunghan.lee@lge.com, Fusion2_JB_Tag-11/06/2012 --
# wooho.jeong@lge.com 2012.11.26
# apply Product
ifeq ($(TARGET_PRODUCT),i_skt)
obj-y += audio_dev_ctl_domestic.o
else ifeq ($(TARGET_PRODUCT),i_lgu)
obj-y += audio_dev_ctl_domestic.o
# mo2sujin.kim, 130116, remove domestic exception for cayman
# else ifeq ($(TARGET_PRODUCT),cayman_skt_kr)
# obj-y += audio_dev_ctl_domestic.o
# else ifeq ($(TARGET_PRODUCT),cayman_kt_kr)
# obj-y += audio_dev_ctl_domestic.o
# else ifeq ($(TARGET_PRODUCT),cayman_lgu_kr)
# obj-y += audio_dev_ctl_domestic.o
#for only Tag !!!
#else ifeq ($(TARGET_PRODUCT),msm8660_surf)		#TEST_CODE - by mo2chunghan.lee@lge.com, Fusion2_JB_Tag-11/06/2012
#obj-y += audio_dev_ctl_domestic.o
else
obj-y += audio_dev_ctl.o
endif
# --------------------------------------------------------
#by mo2chunghan.lee@lge.com, 09/14/2012
#obj-y += board-msm8x60-audio.o
obj-$(CONFIG_TIMPANI_CODEC) += snddev_icodec.o
obj-y += snddev_ecodec.o snddev_mi2s.o snddev_virtual.o
obj-y += pcm_out.o pcm_in.o fm.o
obj-y += audio_lpa.o
#by mo2chunghan.lee@lge.com, Fusion2_JB_Tag-11/06/2012 --
# wooho.jeong@lge.com 2012.11.26
# apply Product
ifeq ($(TARGET_PRODUCT),i_skt)
obj-y += q6voice_domestic.o
else ifeq ($(TARGET_PRODUCT),i_lgu)
obj-y += q6voice_domestic.o
# mo2sujin.kim, 130116, remove domestic exception for cayman
# else ifeq ($(TARGET_PRODUCT),cayman_skt_kr)
# obj-y += q6voice_domestic.o
# else ifeq ($(TARGET_PRODUCT),cayman_kt_kr)
# obj-y += q6voice_domestic.o
# else ifeq ($(TARGET_PRODUCT),cayman_lgu_kr)
# obj-y += q6voice_domestic.o
#for only Tag !!!
#else ifeq ($(TARGET_PRODUCT),msm8660_surf)		#TEST_CODE - by mo2chunghan.lee@lge.com, Fusion2_JB_Tag-11/06/2012
#obj-y += q6voice_domestic.o
else
obj-y += q6voice.o
endif
# --------------------------------------------------------
obj-y += snddev_hdmi.o
obj-y += audio_mvs.o
obj-$(CONFIG_FB_MSM_HDMI_MSM_PANEL) += lpa_if_hdmi.o
endif
obj-$(CONFIG_MSM_QDSP6_APR) += apr.o apr_tal.o q6core.o dsp_debug.o
obj-y += audio_acdb.o
ifdef CONFIG_ARCH_MSM9615
obj-y += rtac.o
endif
obj-$(CONFIG_MSM_QDSP6_CODECS) += aac_in.o qcelp_in.o evrc_in.o amrnb_in.o audio_utils.o
obj-$(CONFIG_MSM_QDSP6_CODECS) += audio_wma.o audio_wmapro.o audio_aac.o audio_multi_aac.o audio_utils_aio.o
obj-$(CONFIG_MSM_QDSP6_CODECS) += rtac.o q6audio_v1.o q6audio_v1_aio.o
obj-$(CONFIG_MSM_QDSP6_CODECS) += audio_mp3.o audio_amrnb.o audio_amrwb.o audio_evrc.o audio_qcelp.o amrwb_in.o
obj-$(CONFIG_MSM_QDSP6V2_CODECS) += aac_in.o qcelp_in.o evrc_in.o amrnb_in.o audio_utils.o
obj-$(CONFIG_MSM_QDSP6V2_CODECS) += audio_wma.o audio_wmapro.o audio_aac.o audio_multi_aac.o audio_utils_aio.o
obj-$(CONFIG_MSM_QDSP6V2_CODECS) += rtac_v2.o q6audio_v2.o q6audio_v2_aio.o
obj-$(CONFIG_MSM_QDSP6V2_CODECS) += audio_mp3.o audio_amrnb.o audio_amrwb.o audio_evrc.o audio_qcelp.o amrwb_in.o
obj-$(CONFIG_MSM_ULTRASOUND) += ultrasound/
