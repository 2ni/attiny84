# INSTALLATION
```
pio init --board esp32dev
```

# Dependencies
- PubSubClient for terminal.cpp
- RFM69 for rfm69_receive.cpp

```
pio lib install PubSubClient RFM69
```

# Run

- terminal.cpp
```
./activate.py modules/common/base.h modules/common/wifi.* modules/common/creds.h modules/common/mqtt.* modules/terminal.cpp
make st
```

- rfm69_receive.cpp
```
./activate.py modules/common/base.h modules/rfm69_receive.cpp
make st
```
