# -*- coding: utf-8 -*-
"""
Created on Thu Dec 10 11:33:38 2015

@author: atanasov2
"""

from os.path import isfile
from exceptions import IOError, KeyError, TypeError
from yaml import YAMLError, load

# Exceptions
class UnknownNodeException(Exception):
    pass

class UnknownCommandException(Exception):
    pass

class TooManyCommands(Exception):
    pass

class MissingCommand(Exception):
    pass

class MissingLaunchFiles(Exception):
    pass

class SR2YamlLoaderFailureException(Exception):
    pass

class MissingComponents(Exception):
    pass

class TooManyFiles(Exception):
    pass



class SR2YamlLoader:
    ''' Converts a SR2Dashboard valid configuration file (YAML) into a format,
    which can then be used for generating the UI of the SR2Dashboard. It does NOT
    create any real UI elements. This is done the SR2UiGenerator'''
    
    class SR2YamlMenu:
        
        class SR2YamlMenuEntry:
            
            class SR2ExecutionUnit:
                def __init__(self, package, command, target):
                    self.package = package
                    self.command = command  # rosrun, roslaunch or rosservice call
                    self.target = target    # rosnode, roslaunch file or service (currently only Trigger services are supported)
                        
            class SR2YamlMenuView:
                
                class SR2YamlMenuViewButton:
                    def __init__(self):
                        self.captions = []
                        self.icons = []
                        self.execute = None
                        
                    
                def __init__(self, buttons):
                    self.buttons = buttons                 
            
            def __init__(self, name, viewFlag, execution_unit=None):
                self.module_name = name
                self.withView = viewFlag
                if viewFlag:
                    self.execute = None
                else:
                    self.execute = execution_unit
                self.entry_icons = []
                pass
            
            def add_icons(self, entry_icons):
                self.entry_icons = entry_icons
            
        def __init__(self):
            self.entries = []
            
        def add_entry(self, entry):
            self.entries.append(entry)

    def __new__(cls, yamlConfigFile):
        
        if not isfile(yamlConfigFile):
            raise IOError('Unable to open configuration file file')
        
        return super(SR2YamlLoader, cls).__new__(cls)
    
    def __init__(self, yamlConfigFile):
        self.config = yamlConfigFile
        self.components = []    # contains all the buttons that will be placed in the SR2YamlGuiWidget
        pass
    
    def get_params_from_server(self):
        # TODO Rewite this to fit the new config file format
        #config = rospy.get_param('/dashboard/menus')
        with open(self.config, 'r') as f:
            try:
                self.yamlData = load(f)
            except YAMLError as exc:
                print('Error parsing YAML file.')
                print('Full message: ', exc)
                return None
            
            ymenus = None
            try:
                menus_all = self.yamlData['dashboard']['menus']                
            except KeyError as e:
                print('Unable to find "menus" node')
                return None
            
            ymenus = []
            try:
                for menu in menus_all:
                    print('Parsing menu')
                    ymenu = SR2YamlLoader.SR2YamlMenu()
                    
                    try:
                        #for i in range(0, len(menu['menu']['modules'])):
                        print(menu['menu']['modules'])
                        #for module in menu['menu']['modules']:
                        #    print('Parsing module')
                    except (KeyError, IOError, UnknownNodeException) as e:
                        print('Unknown node detected')
                        return None
#                    print('Found menu with %d modules' % len(menu['modules']))
#                    ymenu = SR2YamlLoader.SR2YamlMenu()
#                    
#                    try:
#                        for module in menu['modules']:
#                            print('menu :: module["%s"]' % module['name'])
#                            print('module is of type "%s"' % module['type'])  
#                            yentry = SR2YamlLoader.SR2YamlMenu.SR2YamlMenuEntry(
#                                module['name'], 
#                                module['type'] == 'view'
#                            )
#                            
#                            menu_entry = module['menu_entry']
#
#                            entry_icons = [menu_entry['icons']['default']]
#                            # If only default icon (index 0) is given we use it for both states of the menu entry
#                            entry_icons.append(entry_icons[0] if menu_entry['icons']['default'] == '' else menu_entry['icons']['pressed'])
#                                
#                            # Check if icons are valid files
#                            if not isfile(entry_icons[0]):
#                                print('Given path for default icon (menu entry) points at an invalid file. Path will be set to empty')
#                                entry_icons[0] = ''
#                            if not isfile(entry_icons[1]):
#                                print('Given path for pressed icon (menu entry) points at an invalid file. Path will be set to empty')
#                                entry_icons[1] = ''
#                                
#                            if entry_icons[0] == entry_icons[1] and entry_icons[0] == '':
#                                print('Due to both entry icon paths being empty icons list will be set to None')
#                                entry_icons = None
#                            
#                            yentry.add_icons(entry_icons)                     
#                            
#                            if not yentry.withView:
#                                command = ''
#                                execution_unit = SR2YamlLoader.SR2YamlMenu.SR2YamlMenuEntry.SR2ExecutionUnit(menu_entry['rospkg'])
#                            
#                            ymenu.add_entry(yentry)
#                    except (KeyError, IOError, UnknownNodeException) as e:
#                        print(e.message)
#                        return None
#                    
#                    ymenus.append(ymenu)
#                    
            except (KeyError, IOError, UnknownNodeException) as e:
                print('Wooooops!')                
                print(e.message)
                return None
                
        #self.components = []
        
    def getComponents(self):
        return self.components
        
    def getActions(self):
        return self.actions
        
        
yl = SR2YamlLoader('/home/atanasov2/catkin_ws/src/sr2_dashboard/resources/config/config_new_format.yaml')
yl.get_params_from_server()

from pprint import PrettyPrinter # for pretty-printing data structures (remove when finished)
pp = PrettyPrinter(indent=3)
pp.pprint(yl.getComponents())