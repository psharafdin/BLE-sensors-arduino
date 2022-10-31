#include <ArduinoBLE.h>           // Bluetooth Library
#include <Arduino_HTS221.h>       // Temperature Sensor Library
#include <Arduino_LPS22HB.h>      // Pressure Sensor Library
#include <Arduino_LSM9DS1.h>      // Magnetometer Sensor Library


// watchdog timeout in seconds
int wdt = 180;

// Initalizing global variables for sensor data to pass onto BLE
String p, t, h, m;


// The name of the room where the sensor will be installed there. Make sure that the room_name is matched with the room name at the BIM model.
String room_name = "A-1541";

//The device_id should be a uniqe id in the azure IoT hub in the section of devices
String device_id = "A-1541";
void(* resetFunc) (void) = 0;
// BLE Service Name
BLEService EnvironmentalSensing ("290C");

// BLE Characteristics
// Syntax: BLE<DATATYPE>Characteristic <NAME>(<UUID>, <PROPERTIES>, <DATA LENGTH>)
BLEStringCharacteristic ble_pressure("2A6D", BLERead | BLENotify, 13);
BLEStringCharacteristic ble_temperature("2A6E", BLERead | BLENotify, 13);
BLEStringCharacteristic ble_humidity("2A6F", BLERead | BLENotify, 13);
BLEStringCharacteristic ble_magnetic("2AA0 ", BLERead | BLENotify, 20);
BLEStringCharacteristic ble_device_name("2A00", BLERead | BLENotify, 13);
BLEStringCharacteristic ble_location("2AB5", BLERead | BLENotify, 40);


// Function prototype
void readValues();

void setup()
{
    // Initalizing all the sensors
    HTS.begin();
    BARO.begin();
    IMU.begin();
    Serial.begin(115200);
    //while (!Serial);

    //Configure WDT.
    NRF_WDT->CONFIG         = 0x01;             // Configure WDT to run when CPU is asleep
    NRF_WDT->CRV            = wdt * 32768 + 1;  // set timeout
    NRF_WDT->RREN           = 0x01;             // Enable the RR[0] reload register
    NRF_WDT->TASKS_START    = 1;                // Start WDT
    
    
    if (!BLE.begin())
    {
        Serial.println("BLE failed to Initiate");
        delay(500);
        while (1);
    }
    if (!HTS.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
    }

    // Setting BLE Name
    BLE.setLocalName("Arduino Environment Sensor6");
    
    // Setting BLE Service Advertisment
    BLE.setAdvertisedService(EnvironmentalSensing );
    
    // Adding characteristics to BLE Service Advertisment
    EnvironmentalSensing .addCharacteristic(ble_pressure);
    EnvironmentalSensing .addCharacteristic(ble_temperature);
    EnvironmentalSensing .addCharacteristic(ble_humidity);
    EnvironmentalSensing .addCharacteristic(ble_magnetic);
    EnvironmentalSensing .addCharacteristic(ble_device_name);
    EnvironmentalSensing .addCharacteristic(ble_location);

    // Adding the service to the BLE stack
    BLE.addService(EnvironmentalSensing );

    // Start advertising
    BLE.advertise();
    Serial.println("Bluetooth device is now active, waiting for connections...");
}

void loop()
{
    
    // Variable to check if cetral device is connected
    BLEDevice central = BLE.central();
    if (central)
    {
        Serial.print("Connected to central: ");
        Serial.println(central.address());
        while (central.connected())
        {
            delay(5);
            NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reset watch dog timer
            
            // Read values from sensors
            readValues();

            // Writing sensor values to the characteristic
            ble_pressure.writeValue(p);
            ble_temperature.writeValue(t);
            ble_humidity.writeValue(h);
            ble_magnetic.writeValue(m);
            ble_device_name.writeValue(device_id);
            ble_location.writeValue(room_name);
            // Displaying the sensor values on the Serial Monitor
            Serial.println("Reading Sensors");
            Serial.println(p);
            Serial.println(t);
            Serial.println(h);
            Serial.println(m);
            Serial.println("\n");
            delay(5);
        }
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
}


void readValues()
{
    // Reading raw sensor values from three sensors
    float x, y, z;
    float pressure = BARO.readPressure();
    float temperature = HTS.readTemperature();
    float humidity    = HTS.readHumidity();
    if (IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(x, y, z);

    // Saving sensor values into a user presentable way with units
    p = String(pressure);
    t = String(temperature-5);
    h = String(humidity+5);
    m =   String(x) + String(y);
    }
}
