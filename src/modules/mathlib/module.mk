############################################################################
#
#   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#
# Math library
#
SRCS		 = math/test/test.cpp \
		   math/Vector.cpp \
		   math/Vector2f.cpp \
		   math/Vector3.cpp \
		   math/EulerAngles.cpp \
		   math/Quaternion.cpp \
		   math/Dcm.cpp \
		   math/Matrix.cpp \
		   math/Limits.cpp

#
# In order to include .config we first have to save off the
# current makefile name, since app.mk needs it.
#
APP_MAKEFILE	:= $(lastword $(MAKEFILE_LIST))
-include $(TOPDIR)/.config

ifeq ($(CONFIG_ARCH_CORTEXM4)$(CONFIG_ARCH_FPU),yy)
INCLUDE_DIRS	+= math/arm
SRCS		+= math/arm/Vector.cpp \
		   math/arm/Matrix.cpp
else
#INCLUDE_DIRS	+= math/generic
#SRCS		+= math/generic/Vector.cpp \
#		   math/generic/Matrix.cpp
endif
