# Author: Aleksandar Vladimirov Atanasov
# Description: Extracts data from menu_entry or button

import rospy
import rospkg
import os
from rqt_robot_dashboard.util import IconHelper

# For SR2ButtonImageGenerator
import rospkg
from python_qt_binding.QtSvg import QSvgRenderer


class IconType():
    type_view = 0
    type_proc = 1
    type_srv = 2
    type_app = 3
    type_none = 4

    inactive = 0
    running = 1
    error = 2

    @staticmethod
    # change view to icon_type (take type_* values) which will be used to load
    # default icon based on the functionality that the UI component using the
    # image has
    def checkImagePath(icon_path=None, pkg=None, icon_type=type_proc):
        '''
        Checks if the given icon_path represents a valid path
        Note that loading the image may still fail even if the path is valid in
        case the file is not an image supported by Qt's stylesheets

        :param icon_path: path to image that will be used by the button as an icon
        :param pkg: name of package (used in combination with icon_path if icon_path is relative and inside pkg)
        :param view: used to determine default image for icon if icon_path is not valid. Ignored whenever icon_path is valid
        :returns: unchanged icon_path if path is valid, else default path to image for icon
        '''
        rp = rospkg.RosPack()
        error = False

        try:
            if icon_path:
                if icon_path.startswith('~'):
                    icon_path = os.path.expanduser(icon_path)
                if pkg:
                    # If icon_path doesn't represent a valid file, package is taken into consideration
                    # and an attempt is made to generate a valid path using it
                    # and the value of icon_path combined
                    if not os.path.isfile(icon_path):
                        rospy.logdebug(
                            'SR2: Package argument is present. Attempting to get full path of package and generate new path to icon')
                        # In case package parameter is set generate a new
                        # absolute path that includes the package's path
                        pkg_path = rp.get_path(pkg)
                        icon_path = pkg_path + '/' + icon_path
                    else:
                        # Since icon_path is a valid path to a file, package is
                        # ignored
                        rospy.logdebug(
                            'SR2: Package argument is present however given path is an absolute path referencing a file. Package will be ignored')

                # Check if path points to a valid file (also after the package's path has been added to icon_path)
                # Note: That if file is valid since type of file is not checked loading it may still fail)
                #       It is up to the user to give a valid image file
                if not os.path.isfile(icon_path):
                    error = True

            else:
                rospy.logerr('SR2: Given icon path is empty')
                error = True

        except rospkg.ResourceNotFound:
            # If rp.get_path(pkg) fails
            rospy.logerr('SR2: Unable to retrieve path for given package')
            error = True
        except Exception as e:
            rospy.logerr(
                'SR2: Unknown exception occurred during check of path\'s validity.\nFull message: %s', e.message)
            error = True
        finally:
            if error:
                # Fallback to default
                rospy.logwarn(
                    'SR2: Unable to find image at given path "%s". Falling back to default', icon_path)
                sr2path = rp.get_path('sr2_dashboard')
                if icon_type == IconType.type_view:
                    icon_path = sr2path + '/resources/images/default/default_view.svg'
                elif icon_type == IconType.type_proc:
                    icon_path = sr2path + '/resources/images/default/default_proc.svg'
                elif icon_type == IconType.type_srv:
                    icon_path = sr2path + '/resources/images/default/default_service.svg'
                elif icon_type == IconType.type_app:
                    icon_path = sr2path + '/resources/images/default/default_app.svg'
                else:
                    icon_path = sr2path + '/resources/images/default/default_none.svg'

            return icon_path


class SR2PkgCmdExtractor:
    '''
    Returns the dimensions of a grid layout for the distribution of elements of a given list
    '''
    @staticmethod
    def getRosPkgCmdData(yamlEntry, view=False):
        '''
        Extracts the package, command (node->rosrun, launch->roslaunch and service->rosservice call) and arguments (node->ROS node, launch->launch file, serivice->service to call) from YAML menu_entry and button data

        menu_entry:
          package: "lt"--------------------|_____data source
          node: "talker.py"----------------|

        #menu_entry:
          #buttons:
            - captions:
              ...
              package: "lt"----------------|_____data source
              node: "talker.py"------------|
        '''
        if not yamlEntry:
            rospy.logwarn('SR2: Empty entry configuration')
            return ('', '', '', '', 0)

        type = ''
        icon = ''
        icon_path = ''
        icon_type = None
        pkg = ''
        
        ''''
        namespace = ''
        yaml_default = '' #for servces that change param values (using dynamic reconfigure), this is the yaml with the content to be loaded when unpressing the button
        yaml_pressed = '' #the content to be loaded when a reconfigure button is pressed
        '''

        if 'app' in yamlEntry:
            type = 'app'
            icon_type = IconType.type_app
        elif 'launch' in yamlEntry:
            type = 'launch'
            icon_type = IconType.type_proc
        elif 'node' in yamlEntry:
            type = 'node'
            icon_type = IconType.type_proc
        elif 'service' in yamlEntry:
            type = 'service'
            icon_type = IconType.type_srv

        if 'package' in yamlEntry:
            pkg = yamlEntry['package']

        if 'icon' in yamlEntry:
            icon_path = yamlEntry['icon']
        
        icon = IconType.checkImagePath(icon_path, pkg, icon_type)
        
        return (type, icon)
