# sensor-platform-with-homie
C++ code for the Arduino IDE for use on ESP8266 wi-fi connected microprocessors. 

Uses Homie as a base allowing configuration via a json file (see /data/sample-config.json). 
The code includes a range of standard and common sensors that might be used for home automation.

**THIS IS STILL A WORK IN PROGRESS**

# Dependencies

* Homie v2 http://marvinroger.github.io/homie-esp8266/2.0.0-beta.1
- ArduinoJson >= 5.0.8 https://github.com/bblanchon/ArduinoJson
- Bounce2 https://github.com/thomasfredericks/Bounce2
- ESPAsyncTCP >= c8ed544 https://github.com/me-no-dev/ESPAsyncTCP
- AsyncMqttClient https://github.com/marvinroger/async-mqtt-client
* config.json file loaded to the ESP's filing system

# License

This code is Open Source under an Apache 2 License. Please see the [apache2-license.txt file](https://github.com/TotallyInformation/sensor-platform-with-homie/apache2-license.txt) for details.

You may not use this code except in compliance with the License. You may obtain an original copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an
"AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. Please see the
License for the specific language governing permissions and limitations under the License.

# Author

[Julian Knight](https://uk.linkedin.com/in/julianknight2/) ([Totally Information](https://www.totallyinformation.com)), https://github.com/totallyinformation

# Feedback and Support

Please report any issues or suggestions via the [Github Issues list for this repository](https://github.com/TotallyInformation/sensor-platform-with-homie/issues).
