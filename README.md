# powerMeter
Power meter for a bike that computes power, cadence, and resistance similar to a peloton bike.

## powermeterSensor
This is the thing you attach to the bike crank. The key sensor here is the [HX711](https://smile.amazon.com/gp/product/B07MTYT95R) configured in a [wheatstone bridge](powermeterCommon/HX711_circuit.jpg).
```
void setup() {
    set up bluetooth characteristics for read, notify
}
void loop() {
    read sensors
    if values change,
        write to bluetooth characteristics
}
```

## powermeterDisplay
You could display the results on your phone, or another type of display.  For my bike, I wanted to play with some seven segment displays, so I got three [three-digit seven segment displays](https://smile.amazon.com/gp/product/B07GTQS4N5).

Pseudo-code is:
```
void setup() {
    set up bluetooth
}

void loop() {
    connect to bluetooth
    while connected to bluetooth{
        subscribe to notifications
        if values updated
            read values
        display
    }
}
```
