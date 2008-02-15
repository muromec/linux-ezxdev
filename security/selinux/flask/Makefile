#
# Makefile for building the SELinux module as part of the kernel.
#

AWK = awk

CONFIG_SHELL := $(shell if [ -x "$$BASH" ]; then echo $$BASH; \
          else if [ -x /bin/bash ]; then echo /bin/bash; \
          else echo sh; fi ; fi)

FLASK_H_DEPEND = security_classes initial_sids
AV_H_DEPEND = access_vectors
IDIR = ../include/linux/flask

FLASK_H_FILES = av_inherit.h av_perm_to_string.h av_permissions.h class_to_string.h common_perm_to_string.h flask.h initial_sid_to_string.h
FLASK_IH_FILES := $(addprefix $(IDIR)/,$(FLASK_H_FILES))

all:  ../include/linux/flask/flask.h ../include/linux/flask/av_permissions.h

../include/linux/flask/flask.h: $(FLASK_H_DEPEND)
	$(CONFIG_SHELL) ./mkflask.sh $(AWK) $(FLASK_H_DEPEND)
	install flask.h class_to_string.h initial_sid_to_string.h $(IDIR)

../include/linux/flask/av_permissions.h: $(AV_H_DEPEND)
	$(CONFIG_SHELL) ./mkaccess_vector.sh $(AWK) $(AV_H_DEPEND)
	install av_inherit.h common_perm_to_string.h av_perm_to_string.h av_permissions.h $(IDIR)

clean:  
	rm -f $(FLASK_H_FILES)
	rm -f $(FLASK_IH_FILES)

include $(TOPDIR)/Rules.make
