# Makefile for the PCDS gateways startup scripts

# This is the Makefile for the scripts

ifndef EPICS_SITE_CONFIG
$(error EPICS_SITE_CONFIG undefined)
endif

include $(EPICS_SITE_CONFIG)

ifndef INSTALL_LOCATION
INSTALL_LOCATION=.
endif

INSTALL_BIN_DIR=$(INSTALL_LOCATION)/bin
INSTALL_LIB_DIR=$(INSTALL_LOCATION)/lib
INSTALL_ETC_DIR=$(INSTALL_LOCATION)/etc

NETWORKS=cds-amo.pcdsn cds-fee.pcdsn cds.pcdsn dev.pcdsn mcc-epics.pcdsn dev.lab2.reg cds.lab2.reg

# NETWORK_EXPANDED is a list of networks with all domains above (ex if a.b
# is in NETWORKS then a.b AND b will be in NETWORKS_EXPANDED
find_domains=$(if $(findstring .,$(1)),$(1) $(call find_domains, $(shell echo $(1) | sed -e 's|[^\.]*\.||')),$(1))
NETWORKS_EXPANDED=$(sort $(foreach subnet,$(NETWORKS),$(call find_domains, $(subnet))))

ENV_FILES=default.gwenv $(addsuffix .gwenv,$(NETWORKS_EXPANDED))
ACCESS_FILES=$(addsuffix .access,$(NETWORKS))
PVLIST_FILES=$(addsuffix .pvlist,$(NETWORKS))
BIN_FILES=gateway.sh
LIB_FILES=

BUILD_DIR=build
BUILD_ENV_FILES=$(addprefix $(BUILD_DIR)/,$(ENV_FILES))
BUILD_ACCESS_FILES=$(addprefix $(BUILD_DIR)/,$(ACCESS_FILES))
BUILD_PVLIST_FILES=$(addprefix $(BUILD_DIR)/,$(PVLIST_FILES))
BUILD_ETC_FILES=$(BUILD_ENV_FILES) $(BUILD_ACCESS_FILES) $(BUILD_PVLIST_FILES)
BUILD_BIN_FILES=$(addprefix $(BUILD_DIR)/,$(BIN_FILES))
BUILD_LIB_FILES=$(addprefix $(BUILD_DIR)/,$(LIB_FILES))

generate_template_list=$(addsuffix $(suffix $(1)),$(call find_domains, $(notdir $(basename $(1)))) default)

all: install

$(BUILD_DIR):
	@mkdir -p $@

$(BUILD_ENV_FILES): $(BUILD_DIR) force
	@set -e; if [ -e $(notdir $@) ]; then cp $(notdir $@) $@; else echo "CREATING $(notdir $@)"; touch $@; fi

$(BUILD_ACCESS_FILES): $(BUILD_DIR) force
	@set -e; if [ -e $(notdir $@) ]; then \
		cp $(notdir $@) $@; \
	else \
		for template in $(call generate_template_list,$@); do \
			if [ -e $${template} ]; then \
				echo "GENERATE $(notdir $@) (from $${template})"; \
				cp $${template} $@; \
				break; \
			fi; \
		done; \
		if [ ! -e $@ ]; then \
			echo "No template found to generate $(notdir $@) (template list is $(call generate_template_list,$@))"; \
			false; \
		fi; \
	fi

$(BUILD_PVLIST_FILES): $(BUILD_DIR) force
	@set -e; if [ -e $(notdir $@) ]; then \
		cp $(notdir $@) $@; \
	else \
		for template in $(call generate_template_list,$@); do \
			if [ -e $${template} ]; then \
				echo "GENERATE $(notdir $@) (from $${template})"; \
				cp $${template} $@; \
				break; \
			fi; \
		done; \
		if [ ! -e $@ ]; then \
			echo "No template found to generate $(notdir $@) (template list is $(call generate_template_list,$@))"; \
			false; \
		fi; \
	fi

$(BUILD_BIN_FILES) $(BUILD_LIB_FILES): $(BUILD_DIR) force
	@set -e; if [ -e $(notdir $@) ]; then cp $(notdir $@) $@; else false; fi

install: $(BUILD_ENV_FILES) $(BUILD_ACCESS_FILES) $(BUILD_PVLIST_FILES) $(BUILD_BIN_FILES) $(BUILD_LIB_FILES)
	@set -e; if [ ! -d $(INSTALL_BIN_DIR) ]; then mkdir -p $(INSTALL_BIN_DIR); fi
	@set -e; for file in $(BUILD_BIN_FILES); do \
		echo "INSTALL $$(basename $${file}) (bin)"; \
		cp $${file} $(INSTALL_BIN_DIR)/$$(basename $${file}); \
	done
	@set -e; if [ ! -d $(INSTALL_LIB_DIR) ]; then mkdir -p $(INSTALL_LIB_DIR); fi
	@set -e; for file in $(BUILD_LIB_FILES); do \
		echo "INSTALL $$(basename $${file}) (bin)"; \
		cp $${file} $(INSTALL_LIB_DIR)/$$(basename $${file}); \
	done
	@set -e; if [ ! -d $(INSTALL_ETC_DIR) ]; then mkdir -p $(INSTALL_ETC_DIR); fi
	@set -e; for file in $(BUILD_ETC_FILES); do \
		echo "INSTALL $$(basename $${file}) (bin)"; \
		cp $${file} $(INSTALL_ETC_DIR)/$$(basename $${file}); \
	done
	@echo "INSTALL init.d scripts"
	@set -e; if [ ! -d $(INSTALL_ETC_DIR)/init.d ]; then mkdir -p $(INSTALL_ETC_DIR)/init.d; fi
	@cp epics-gateway $(INSTALL_ETC_DIR)/init.d/
	@set -e; if [ ! -d $(INSTALL_ETC_DIR)/sysconfig ]; then mkdir -p $(INSTALL_ETC_DIR)/sysconfig; fi
	@cp epics-gateway.cfg $(INSTALL_ETC_DIR)/sysconfig/

clean:
	@set -e; if [ -d $(BUILD_DIR) ]; then rm -r $(BUILD_DIR); fi

distclean realclean: clean
	@set -e; \
	for dir in $(INSTALL_LIB_DIR) $(INSTALL_ETC_DIR) $(INSTALL_BIN_DIR); do \
		if [ -d $${dir} ]; then \
			echo "REMOVE $${dir}"; \
			rm -r $${dir}; \
		fi; \
	done

force:

.PHONY: all install clean distclean realclean force
