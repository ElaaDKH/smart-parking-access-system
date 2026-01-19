#include <WiFi.h>
#include <HTTPClient.h>

// WiFi credentials
#define WIFI_SSID "Galaxy A23 FA4C"
#define WIFI_PASS "elaaelaa"

// Python server
#define PYTHON_IP "192.168.247.162"
#define PYTHON_PORT 5000

void setup() {
  //Serial Monitor 
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== ESP32-S3 PARKING SYSTEM ===");
  
  //UART pour STM32
  Serial1.begin(115200, SERIAL_8N1, 44, 43);  // RX=44, TX=43
  
  //Connexion WiFi
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("\nREADY - Waiting for CAPTURE command from STM32\n");
}

void loop() {
  //Attendre commande du STM32
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();
    
    Serial.print(">>> RX FROM STM32: ");
    Serial.println(command);
    
    if (command == "CAPTURE") {
      Serial.println("=== Processing Capture Request ===");
      
      //Appeler le serveur Python
      String url = "http://" + String(PYTHON_IP) + ":" + String(PYTHON_PORT) + "/capture";
      Serial.print("Requesting: ");
      Serial.println(url);
      
      HTTPClient http;
      http.begin(url);
      http.setTimeout(120000);  //120 secondes
      
      int httpCode = http.GET();
      
      Serial.print("HTTP Response Code: ");
      Serial.println(httpCode);
      
      if (httpCode == 200) {
        String response = http.getString();
        Serial.print("Response: ");
        Serial.println(response);
        delay(500);
        
        //Envoyer au STM32
        for(int i = 0; i < response.length(); i++) {
        Serial1.write(response[i]);
        delayMicroseconds(1000);  
  }
       Serial1.write('\n');
       Serial1.flush();
        
        Serial.println(">>> TX TO STM32: " + response);
      } else {
        //Erreur HTTP
        String error = "{\"status\":\"error\",\"authorized\":false}\n";
        Serial1.print(error);
        Serial1.flush();
        
        Serial.print("HTTP Error: ");
        Serial.println(httpCode);
      }
      
      http.end();
      Serial.println("=== Request Complete ===\n");
    }
  }
  
  delay(10);
}
