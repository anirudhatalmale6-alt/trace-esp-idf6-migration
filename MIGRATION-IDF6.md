# Trace - ESP-IDF 5.x to 6.0 migration notes

Target: **esp32c3**. Verified against **ESP-IDF v6.0.2** (the same version your
`.vscode/settings.json` points at: `C:\esp\v6.0.2\esp-idf`).

Result: `idf.py build` completes from a clean tree, compiles and links.

```
trace.bin binary size 0xe24c0 bytes. Smallest app partition is 0x177000 bytes.
0x94b40 bytes (40%) free.
Project build complete.
```

Every change is tagged in the source with `TODO(IDF6):`. To find them all:

```
grep -rn "TODO(IDF6)" main CMakeLists.txt
```

There are 21 such markers. Most are explanatory - they tell you what moved and
why. **Three need a decision from you**; they are listed under "Needs your
review" below.

---

## Needs your review

### 1. `main/IMU.c` - two disabled calls in `trace_build()` (lines ~206 and ~252)

```c
ICM45686_convert_to_g(FIFO_return_next(), &current);
ICM45686_convert_to_g(FIFO_return_previous(), &current);
```

`FIFO_return_next()` / `FIFO_return_previous()` return `FIFO_raw_t *`.
`ICM45686_convert_to_g()` takes a `FIFO_fixed_single_t *`. The raw FIFO frame
has to be unpacked first - the pipeline is

```
FIFO_raw_t  ->  ICM45686_fixed_unpack()  ->  FIFO_fixed_single_t
            ->  ICM45686_convert_to_g()  ->  trace_vector_t
```

The unpack step is simply missing. Older compilers let this through as a
warning; the 6.0 toolchain rejects it.

I have **not** written the missing step, because there are two unpack variants
(`ICM45686_fixed_unpack()` and `ICM45686_real_unpack()`) with different scaling,
and choosing wrong would quietly corrupt the trace maths rather than fail
loudly. This is your call. Roughly what it wants:

```c
FIFO_fixed_single_t frame;
ICM45686_fixed_unpack(FIFO_return_next(), &frame);
ICM45686_convert_to_g(&frame, &current);
```

Both calls are commented out, so `trace_build()` compiles but the two loops no
longer compute anything. `current` is zero-initialised at its declaration purely
so the compiler stops warning that it is read before being written - remove the
`= {0}` when you restore the calls.

### 2. `main/nonvol.c` line ~353 - `nonvol_write_i32()` stored the pointer

```c
/* was */ nvs_set_i32(my_handle, name, value);    /* value is an int *  */
/* now */ nvs_set_i32(my_handle, name, *value);
```

`nvs_set_i32()` takes an `int32_t` **value**, not a pointer. As written, what
went into NVS was the *address* of the variable, not its contents. GCC 15 makes
this a hard error ("makes integer from pointer without a cast") where it used to
be a warning.

I dereferenced it, which matches the function's name and signature - but this is
a genuine bug rather than a 6.0 rename, so please confirm that is what you
wanted. Note it also means the values previously written to NVS under this path
were meaningless, so a factory reset may be in order once you are running again.

### 3. `main/drivers/ICM45686.c` line ~577 - wrong unpack variant in `ICM45686_find_zero()`

```c
/* was */ ICM45686_fixed_unpack(&FIFO_queue[index_in.outer].f[i], &sample);
/* now */ ICM45686_real_unpack (&FIFO_queue[index_in.outer].f[i], &sample);
```

Every local in that function - `sample`, `min_sample`, `max_sample` - is
declared `FIFO_real_single_t`, while `fixed_unpack()` wants a
`FIFO_fixed_single_t *`, so the call never matched its arguments. I switched to
the `_real_` variant because that is the one whose signature matches the
declarations. Please confirm that was the intent rather than the locals being
the thing that is wrong.

---

## What actually broke, and why

### Component reorganisation - the big one

IDF 6.0 split the monolithic `driver` and `hal` components into per-peripheral
components. Comparing the two trees side by side, **142 public headers that
exist in 5.5 are gone from 6.0**, and 6.0 ships 141 components where 5.5 had
far fewer.

The one that bit this project directly:

| header | ESP-IDF 5.5.1 | ESP-IDF 6.0.2 |
|---|---|---|
| `hal/gpio_types.h` | `components/hal/include/hal/` | `components/esp_hal_gpio/include/hal/` |

`main/CMakeLists.txt` listed `hal` in `REQUIRES`, and `hal` no longer provides
that header. Same story for `esp_hal_pcnt`, `esp_hal_i2c`, `esp_hal_ledc`,
`esp_hal_rmt` and the rest of the new `esp_hal_*` family.

`REQUIRES` was also missing most of what the code actually uses. It was:

```
log esp_timer esp_eth esp_http_server esp_adc freertos hal lwip
```

Added: `nvs_flash`, `esp_wifi`, `esp_netif`, `esp_event`, `esp_http_client`,
`esp-tls`, `esp_hw_support`, `app_update`, `driver`, `esp_driver_gpio`,
`esp_driver_spi`, `esp_driver_ledc`, `esp_driver_rmt`, `esp_driver_pcnt`,
`esp_driver_uart`, `esp_hal_gpio`.

### Include paths

Backslash separators - `#include "driver\gpio.h"` - in `main/drivers/gpio.c`,
`main/trace.c` and `main/helpers.c`, plus two commented-out ones in `timer.c`
and `NTP.c`. Changed to forward slashes.

Bare filenames missing their component sub-directory. These headers live at
`<component>/include/<subdir>/<name>.h`, so the sub-directory is part of the
header name:

| was | now |
|---|---|
| `ledc.h` | `driver/ledc.h` |
| `pulse_cnt.h` | `driver/pulse_cnt.h` |
| `spi_master.h` | `driver/spi_master.h` |
| `spi_common.h` | `driver/spi_common.h` |
| `rmt_tx.h` | `driver/rmt_tx.h` |
| `gpio_types.h` | `hal/gpio_types.h` |
| `FreeRTOS.h` | `freertos/FreeRTOS.h` |
| `event_groups.h` | `freertos/event_groups.h` |
| `task.h` | `freertos/task.h` |
| `mpu_wrappers.h` | `freertos/mpu_wrappers.h` |
| `dns.h` | `lwip/dns.h` |

Two case-only mismatches that work on Windows but not on a case-sensitive
filesystem: `ntp.h` -> `NTP.h` in `common.h`, `wifi.h` -> `WiFi.h` in
`diag_tools.c`. `INCLUDE_DIRS` also had `"./tcpip"` where the directory is
`TCPIP`.

Missing includes that used to arrive transitively: `nvs.h` in `nonvol.h` (for
`nvs_handle_t`), `driver/gpio.h` in `drivers/gpio.h` (for `GPIO_NUM_*`) and in
`diag_tools.c` (for `gpio_get_level()` / `gpio_set_level()`).

### The compiler is now GCC 15, building as C23

This is the part that catches people out. IDF 6.0 compiles with
`-std=gnu23 -Wall -Werror -Wextra`. Several long-standing warnings are now
errors:

**Empty parameter lists mean `(void)` in C23.** `http_server.h` declared
`void http_send_string_end();` while `http_server.c` defines it as
`(httpd_req_t *req)`. Under C23 those genuinely conflict. Same problem with
`timer.h`'s `void *(callback)()`, which also had the wrong return type - the
`timers[]` struct in `timer.c` declares the field as `void (*)(void)` and calls
it that way, so the prototype was simply wrong.

**`-Werror=ignored-qualifiers`.** `time_count_64_t` is
`typedef volatile int64_t`, and a `volatile` qualifier on a *return type* is
meaningless, so the compiler discards it and now complains. The six functions
returning it (`run_time_us/ms/s`, `NTP_time_us/ms/s`) now return `int64_t`.
The typedef itself is untouched - variables declared with it are still
`volatile`, which is what matters.

**`-Werror=cast-function-type`.** Casting `bool (*)(void)` straight to
`gpio_isr_t` (`void (*)(void *)`) is rejected outright. Routed via `(void *)`,
which is the usual way to keep it. Generated code is unchanged.

**`-Werror=incompatible-pointer-types`.** The `JSON[]` dispatch table in
`json.c` deliberately stores several different pointer types in two generic
columns. Some rows were already cast, most were not. Added the same explicit
casts to the rest - the table is byte for byte what it was before.

**`-Werror=implicit-fallthrough`.** Three deliberate `switch` fall-throughs in
`json.c` - the comments in the source already said "fall through", the compiler
just wants it stated in a form it can check. Marked with
`__attribute__((fallthrough))`, which emits no code.

**`-Werror` on macro redefinition.** Both `trace.h` and `json.h` define
`EXTERN`, so whichever was included second redefined it. Added `#undef EXTERN`
before each definition, which preserves the existing behaviour exactly.

### Build system

`main/CMakeLists.txt` globbed `${CMAKE_SOURCE_DIR}/main/*.*`, which swept every
file with a dot in its name into `SRCS` - `.h`, `.html`, `.png`, `.json`,
`.pem`, `.js` and the stray `main/build/CMakeLists.txt`. Narrowed to `*.c`.

`main/drivers/server .c` had a **space in the filename**. Renamed to
`server.c`. Its file header says "WiFi.c", which makes it look like a duplicate
of `WiFi.c`, but it is not - `server_send()`, `server_receive_poll()`,
`server_socket_poll_0..3()`, `server_accept_poll()` and
`WiFi_show_connections()` are defined there and nowhere else. `serial_io.c`
calls `server_send()`, so leaving it out breaks the link.

`main/drivers/sockets.c` is **excluded from the build**. It is a verbatim copy
of lwIP's own `sockets.c` - 4613 lines, "This file is part of the lwIP TCP/IP
stack" in the header. It `#include`s `api_lib.c`, `api_msg.c` and `netbuf.c`,
which are lwIP-internal sources not on any include path, and every symbol in it
collides with the real lwIP linked in via the `lwip` component. It looks like a
reference copy you dropped in to read rather than something meant to compile.
If you actually intended to override lwIP's socket layer, that needs doing a
different way - worth a conversation first.

The top-level `CMakeLists.txt` set

```cmake
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)
```

which is leftover boilerplate from the IDF example this project started from.
Nothing under `main/` references it - no `example_connect()`, no `esp_eth`, no
`ethernet_init` - but it drags in the `espressif/ethernet_init` managed
component, whose version pinned in `dependencies.lock` predates 6.0 and fails to
find `esp_eth_driver.h`. Removed, along with the stale `dependencies.lock` and
`managed_components/`. Both regenerate on demand if you ever add a real managed
dependency.

---

## Loose ends worth knowing about

* `main/drivers/WiFi.c` line ~147 calls `mdns_hostname_set()` while the
  `#include "mdns.h"` on line 29 is commented out. It links today only because
  nothing reaches that code path at link time - but `mdns` is no longer part of
  IDF core, it is a separate managed component. If you re-enable mDNS you will
  need to add `espressif/mdns` to `main/idf_component.yml`.
* `.vscode/settings.json` has `"idf.adapterTargetName": "esp32s3"` while
  `sdkconfig` says `esp32c3`. That only affects the debugger, not the build, but
  it will confuse you at the worst possible moment.
* `main/build/CMakeLists.txt` is a stray copy of the top-level project file
  sitting inside `main/`. Harmless now that the glob is restricted to `*.c`, but
  it serves no purpose.
* `cmake_minimum_required(VERSION 3.16)` still works, but IDF 6.0 itself wants
  newer. Not urgent.

---

## Rebuilding on your machine

```
idf.py fullclean
idf.py set-target esp32c3
idf.py build
```

`fullclean` matters - the `build/` directory carries cached CMake state from the
5.x toolchain, and stale cache produces errors that look like source problems.
