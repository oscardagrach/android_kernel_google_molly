dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb

targets += dtbs
dtb-$(CONFIG_MACH_EXUMA) += tegra132-exuma.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-norrin.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-norrin-pm374-0001-a00-00.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-norrin-pm374-0001-a01-00.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-laguna.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-bowmore-ers-s.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-bowmore-e1973-1000-a01-00.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-bowmore-ers.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-bowmore-e1971-1100-a00-00.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-bowmore-e1971-1100-a00-00-powerconfig.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-tn8-p1761-1270-a03-battery.dtb
dtb-$(CONFIG_MACH_T132REF) += tegra132-tn8-p1761-1270-a03.dtb
dtb-$(CONFIG_MACH_T132_FLOUNDER) += tegra132-flounder64.dtb

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif
targets += $(DTB_LIST)

DTC_FLAGS := -i $(srctree)/arch/arm/boot/dts

DTCCPP_FLAGS := -I$(srctree)/arch/arm/boot/dts

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := *.dtb
