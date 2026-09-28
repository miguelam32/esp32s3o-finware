for f in managed_components/espressif__led_strip/src/*.c; do
  grep -q esp_heap_caps.h "$f" || sed -i '1i #include "esp_heap_caps.h"' "$f"
done
