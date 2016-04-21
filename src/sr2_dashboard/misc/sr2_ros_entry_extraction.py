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

    # TODO Remove this once support of custom icons is added for view entries
    @staticmethod
    def loadIcons(name, with_view=False):
        '''
        Loads predefined icons
        '''
        # Create paths
        icon_paths = [['sr2_dashboard', 'resources/images']]
        paths = []
        rp = rospkg.RosPack()
        for path in icon_paths:
            paths.append(os.path.join(rp.get_path(path[0]), path[1]))

        icon_helper = IconHelper(paths, name)

        icons = []
        res_icons = None
        converted_icons = []

        # Add icons
        if with_view:
            # Inactive view
            icons.append(['control/menu/diagnostics_inactive.png'])
            # Active view
            icons.append(['control/menu/diagnostics_running.png'])
            icons.append(['control/menu/diagnostics_error.png'])      # Failed
            rospy.logdebug('SR2: Loaded icons for component with view')
        else:
            icons.append(['status/status_inactive.svg'])         # Inactive
            icons.append(['status/status_running.svg'])          # Active
            icons.append(['status/status_error.svg'])            # Failed
            rospy.logdebug('SR2: Loaded icons for component without view')

        res_icons = list(icons)
        converted_icons = icon_helper.set_icon_lists(icons)

        #converted_clicked_icons = None

        return (converted_icons[0], res_icons)

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

        pkg = ''
        cmd = ''   # Can be rosrun/roslaunch/rosservice call
        # Args is the actual ROS node/launch file/etc. we want to start
        # (exception: see "app" case)
        args = ''
        icon = ''
        icon_path = ''
        icon_type = None
        timeout = 0

        if 'package' in yamlEntry:
            # We can have either a rosrun or roslaunch
            pkg = yamlEntry['package']
            rospy.logdebug('SR2: Found package "%s"', pkg)
            if 'node' in yamlEntry:
                # We have a rosrun command
                cmd = 'rosrun'
                args = yamlEntry['node']        # rosrun pkg node
                rospy.logdebug('SR2: Found rosrun command for node "%s"', args)
                icon_type = IconType.type_proc
            elif 'launch' in yamlEntry:
                # We have a roslaunch command
                cmd = 'roslaunch'
                args = yamlEntry['launch']      # roslaunch pkg launch_file
                if '.launch' not in args:
                    args += '.launch'
                icon_type = IconType.type_proc
                rospy.logdebug(
                    'SR2: Found roslaunch command for launch file "%s"', args)

            if 'args' in yamlEntry:
                # [rosrun/roslaunch] pkg [node/launch_file] arg1:=... arg2:=...
                args1 = yamlEntry['args']
                rospy.logdebug('SR2: Found arguments for command "%s"', args1)
                args = args + args1
            timeout = 0
        elif 'service' in yamlEntry:
            # We have a service
            # my_service becomes /my_service so that it can easily be combined
            # later on with rosservice call
            args = yamlEntry['service']
            rospy.logdebug('SR2: Found service "%s"', args)
            args = '/' + args
            icon_type = IconType.type_srv
            if 'timeout' in yamlEntry:
                try:
                    timeout = int(yamlEntry['timeout'])
                    rospy.logdebug(
                        'SR2: Found timeout for service: %d', timeout)
                    if not timeout or timeout < 0:
                        rospy.logwarn(
                            'SR2: Timeout for service is either negative or equal zero. Falling back to default: 5')
                        timeout = 5
                except:
                    rospy.logwarn(
                        'SR2: Found timeout for service but unable to parse it. Falling back to default: 5')
                    timeout = 5
            else:
                timeout = 5
        elif 'app' in yamlEntry:
            # We have a standalone application
            cmd = yamlEntry['app']
            rospy.logdebug('SR2: Found standalone application "%s"', cmd)
            if 'args' in yamlEntry:
                args = yamlEntry['args']
                rospy.logdebug(
                    'SR2: Found arguments for standalone application "%s"', args)
            timeout = 0
            icon_type = IconType.type_app
        else:
            rospy.logerr('SR2: Unable to parse YAML node:\n%s', yamlEntry)
            icon_type = IconType.type_none

        if 'icon' in yamlEntry:
            icon_path = yamlEntry['icon']
        icon = IconType.checkImagePath(icon_path, pkg, icon_type)
        return (pkg, cmd, args, icon, timeout)
