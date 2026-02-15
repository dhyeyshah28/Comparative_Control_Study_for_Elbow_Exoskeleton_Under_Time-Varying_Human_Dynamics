
#include <WiFi.h>

char ssid[] = "Dhyey";
char pass[] = "dhyeyshah28";

WiFiServer server(5000);  // Port number

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nConnected! IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected");

    while (client.connected()) {
      if (client.available()) {
        String msg = client.readStringUntil('\n');
        Serial.print("Received: ");
        Serial.println(msg);
      }
    }

    Serial.println("Client disconnected");
  }
}
