# Author: Aleksandar Vladimirov Atanasov
# Description: Loads the configuration for the SR2 Dashboard from a yaml file

```
YAML SR2 Dashboard Configuration Specification:
-----------------------------------------------
modules:
	- module:
		- name: "SR2 Emergency Stop"
		- package: "sr2_emergency_stop_widget"
		- widget: "SR2EmergencyStop"
```

def loadSR2DConfig(path):
  configFile = yaml.load(path)
  configFile["modules"]["module"]["
