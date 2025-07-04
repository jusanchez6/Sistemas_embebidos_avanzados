# ESP32 Access Point Web Server - Robot Control Example

This project is the simplest example for an HTTP server on the ESP32 running in **Access Point (AP) mode**. It allows a client device (phone or PC) to connect to the ESP32 Wi-Fi and control robot movement via HTTP GET requests with parameters.

The AP is usually initialized with the IP: `192.168.4.1`

---

## Project Structure

This program is contained in a single `main.c` file. It uses:

* ESP-IDF framework (v5+)
* WiFi in AP mode
* HTTP server (`esp_http_server`)
* URI handlers for different movement commands:

  * `/lineMovement`
  * `/circularMovement`
  * `/selfRotation`
* A companion `index.html` file to generate the query strings from a GUI

---

## How to configure, build and flash

### 1. Wi-Fi SoftAP Initialization

The call to the function `wifi_init_softap();` creates a Wi-Fi Access Point with the credentials:

```c
#define AP_SSID "esp32"
#define AP_PASSWORD "sanchez06"
```

It starts the network stack so that client devices can connect. Once connected, your device will typically get an IP like `192.168.4.x`.

You can test the HTTP endpoint via browser using a direct query like:

```html
http://192.168.4.1/lineMovement?direction=Forward&degrees=90&velocity=20&distance=100
```

Or use the local `index.html` file in your browser to submit parameters via a form.

### 2. Build and Flash

In your ESP-IDF project root directory:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

### 3. Expected Serial Output

After flashing and running, you should see serial output similar to:

```
I (435) WebServer: Initializing WiFi in Access Point mode...
I (1115) WebServer: AP started. SSID: esp32 | Password: sanchez06
I (1135) WebServer: Web server started in AP mode
I (xxxxxx) WebServer: >> Entered handler /lineMovement
I (xxxxxx) WebServer: Query string: direction=Forward&degrees=90&velocity=20&distance=100
I (xxxxxx) WebServer: >> Params OK

```

If any parameters are missing, it will print:

```
E (xxxxxx) WebServer: Error obtaining parameters
```

---

## Key Handlers and Parameters

Each route (`/lineMovement`, `/circularMovement`, `/selfRotation`) has a dedicated handler that:

* Parses GET parameters from the URL query string
* Validates them
* Sends back an acknowledgment message

Example:

```c
/lineMovement?direction=Forward&degrees=90&velocity=30&distance=50
```

The response will be plain text: `Command received`.

If any parameter is missing or invalid, the server responds with:

```http
HTTP/1.1 400 Bad Request - Missing params
```

---

## Notes and Recommendations

* Make sure to enable **CORS headers** if you're using browser-based clients
* Ensure that query keys match **exactly** what the ESP32 handler expects (case-sensitive)
* Use proper error handling in your web frontend to give user feedback
* You may expand this basic logic to control real motors or servos using GPIOs or PWM

---

## Developed by

**Julian Sanchez**
ESP32-S3 / July 2025
