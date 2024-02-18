// Copyright 2021 Víctor Mayoral-Vilches
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tracetools_image_pipeline/tracetools.h"

#ifndef TRACETOOLS_DISABLED

#ifdef TRACETOOLS_LTTNG_ENABLED
# include "tracetools_image_pipeline/tp_call.h"
# define CONDITIONAL_TP(...) \
  tracepoint(TRACEPOINT_PROVIDER, __VA_ARGS__)
#else
# define CONDITIONAL_TP(...)
#endif

bool ros_trace_compile_status()
{
#ifdef TRACETOOLS_LTTNG_ENABLED
  return true;
#else
  return false;
#endif
}

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#else
# pragma warning(push)
# pragma warning(disable: 4100)
#endif


void TRACEPOINT(
  image_proc_resize_init,
  const void * resize_node_arg,
  const void * resize_image_msg_arg,
  const void * resize_info_msg_arg)
{
  CONDITIONAL_TP(
    image_proc_resize_init,
    resize_node_arg,
    resize_image_msg_arg,
    resize_info_msg_arg);
}

void TRACEPOINT(
  image_proc_resize_fini,
  const void * resize_node_arg,
  const void * resize_image_msg_arg,
  const void * resize_info_msg_arg)
{
  CONDITIONAL_TP(
    image_proc_resize_fini,
    resize_node_arg,
    resize_image_msg_arg,
    resize_info_msg_arg);
}

void TRACEPOINT(
  image_proc_rectify_init,
  const void * rectify_node_arg,
  const void * rectify_image_msg_arg,
  const void * rectify_info_msg_arg)
{
  CONDITIONAL_TP(
    image_proc_rectify_init,
    rectify_node_arg,
    rectify_image_msg_arg,
    rectify_info_msg_arg);
}

void TRACEPOINT(
  image_proc_rectify_fini,
  const void * rectify_node_arg,
  const void * rectify_image_msg_arg,
  const void * rectify_info_msg_arg)
{
  CONDITIONAL_TP(
    image_proc_rectify_fini,
    rectify_node_arg,
    rectify_image_msg_arg,
    rectify_info_msg_arg);
}

#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

#endif  // TRACETOOLS_DISABLED
