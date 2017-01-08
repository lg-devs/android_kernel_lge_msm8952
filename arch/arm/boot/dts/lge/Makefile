ifeq ($(CONFIG_OF),y)

ifeq ($(CONFIG_MACH_MSM8952_B3_OPEN_KR),y)
DTS_TARGET := msm8952-b3_open_kr
endif

ifeq ($(CONFIG_MACH_MSM8952_P1V_ATT_US),y)
DTS_TARGET := msm8952-p1v_att_us
endif

ifeq ($(CONFIG_MACH_MSM8952_B3_ATT_US),y)
DTS_TARGET := msm8952-b3_att_us
endif

ifeq ($(CONFIG_MACH_MSM8952_B3_TMO_US),y)
DTS_TARGET := msm8952-b3_tmo_us
endif

ifeq ($(CONFIG_MACH_MSM8952_B3_RGS_CA),y)
DTS_TARGET := msm8952-b3_rgs_ca
endif

ifeq ($(CONFIG_MACH_MSM8952_B3_BELL_CA),y)
DTS_TARGET := msm8952-b3_bell_ca
endif

ifeq ($(CONFIG_MACH_MSM8952_B3_JP_KDI),y)
DTS_TARGET := msm8952-b3_jp_kdi
endif

ifeq ($(CONFIG_MACH_MSM8952_B5_OPEN_KR),y)
DTS_TARGET := msm8952-b5_open_kr
endif

ifeq ($(CONFIG_MACH_MSM8952_B5_ATT_US),y)
DTS_TARGET := msm8952-b5_att_us
endif

ifeq ($(CONFIG_MACH_MSM8952_B5_USC_US),y)
DTS_TARGET := msm8952-b5_usc_us
endif

ifeq ($(CONFIG_MACH_MSM8952_B5_ACG_US),y)
DTS_TARGET := msm8952-b5_acg_us
endif

ifeq ($(CONFIG_MACH_MSM8952_B5_OPEN_US),y)
DTS_TARGET := msm8952-b5_open_us
endif

ifeq ($(CONFIG_MACH_MSM8952_B5_JP_KDI),y)
DTS_TARGET := msm8952-b5_jp_kdi
endif

ALL_DTS := $(notdir $(wildcard echo $(srctree)/$(src)/$(DTS_TARGET)/*.dts))
dtb-y += $(patsubst %.dts,%.dtb,$(ALL_DTS))

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

targets += dtbs
targets += $(addprefix ../, $(DTB_LIST))
endif

$(obj)/../%.dtb: $(src)/$(DTS_TARGET)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(DTB_LIST))

clean-files := *.dtb
