# Author: Aleksandar Vladimirov Atanasov
# Description: Extracts data from menu_entry or button

import rospy
import rospkg
import os
from rqt_robot_dashboard.util import IconHelper

class IconType():
  inactive = 0
  running = 1
  error = 2

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
      icons.append(['control/menu/diagnostics_inactive.png'])   # Inactive view
      icons.append(['control/menu/diagnostics_running.png'])    # Active view
      icons.append(['control/menu/diagnostics_error.png'])      # Failed
      rospy.loginfo('Loaded icons for component with view')
    else:
      icons.append(['status/status_inactive.svg'])         # Inactive
      icons.append(['status/status_running.svg'])          # Active
      icons.append(['status/status_error.svg'])            # Failed
      rospy.loginfo('Loaded icons for component without view')

    res_icons = list(icons)
    converted_icons = icon_helper.set_icon_lists(icons)

    #converted_clicked_icons = None

    return (converted_icons[0], res_icons)

class SR2PkgCmdExtractor:
  '''
  Returns the dimensions of a grid layout for the distribution of elements of a given list
  '''
  @staticmethod
  def getRosPkgCmdData(yamlEntry):
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
      return ('','','', 0)

    pkg = ''
    cmd = ''   # Can be rosrun/roslaunch/rosservice call
    args = ''  # Args is the actual ROS node/launch file/etc. we want to start (exception: see "app" case)
    timeout = 0

    print(yamlEntry)
    #Try each of the possible configurations: node, launch and service

    if 'package' in yamlEntry:
      # We can have either a rosrun or roslaunch
      pkg = yamlEntry['package']
      rospy.loginfo('SR2: Found package "%s"', pkg)
      if 'node' in yamlEntry:
        # We have a rosrun command
        cmd = 'rosrun'
        args = yamlEntry['node']              # rosrun pkg node
        rospy.loginfo('SR2: Found rosrun command for node "%s"', args)
      elif 'launch' in yamlEntry:
        # We have a roslaunch command
        cmd = 'roslaunch'
        args = yamlEntry['launch']            # roslaunch pkg launch_file
        rospy.loginfo('SR2: Found roslaunch command for launch file "%s"', args)
      if 'args' in yamlEntry:
        args1 = yamlEntry['args']       # [rosrun/roslaunch] pkg [node/launch_file] arg1:=... arg2:=...
        rospy.loginfo('SR2: Found arguments for command "%s"', args1)
        args = args + args1
      timeout = 0
    elif 'service' in yamlEntry:
      # We have a service
      args = yamlEntry['service']       # my_service becomes /my_service so that it can easily be combined later on with rosservice call
      rospy.loginfo('SR2: Found service "%s"', args)
      args = '/' + args
      if 'timeout' in yamlEntry:
        try:
          timeout = int(yamlEntry['timeout'])
          rospy.loginfo('SR2: Found timeout for service: %d', timeout)
          if not timeout or timeout < 0:
            rospy.loginfo('SR2: Timeout for service is either negative or equal zero. Falling back to default: 5')
            timeout = 5
        except:
          rospy.logwarn('SR2: Found timeout for service but unable to parse it. Falling back to default: 5')
          timeout = 5
      else:
        timeout = 5
    elif 'app' in yamlEntry:
      # We have a standalone application
      cmd = yamlEntry['app']
      rospy.loginfo('SR2: Found standalone application "%s"', cmd)
      if 'args' in yamlEntry:
        args = yamlEntry['args']
        rospy.loginfo('SR2: Found arguments for standalone application "%s"', args)
      timeout = 0
    else:
      print(yamlEntry)
      rospy.loginfo('SR2: Unable to parse YAML node')

    return (pkg, cmd, args, timeout)

#    try:
#      pkg = yamlEntry['package']
#      rospy.loginfo('SR2: Using package %s' % pkg)
#      try:
#        args = yamlEntry['node']
#        rospy.loginfo('SR2: Nodes detected. Will use "rosrun"')
#        cmd = 'rosrun'
#
#      except KeyError:
#        try:
#          args = yamlEntry['launch'] + '.launch'
#          rospy.loginfo('SR2: Launch file detected. Will use "roslaunch"')
#          cmd = 'roslaunch'
#
#        except KeyError:
#          try:
#            args = yamlEntry['service']
#            rospy.loginfo('SR2: Service deteceted. Will use "rosservice call"')
#            #cmd = 'rosservice call'
#            if 'timeout' in yamlEntry:
#              try:
#                timeout = int(yamlEntry['timeout'])
#              except:
#                rospy.logerr('SR2: Timeout found however not an integer. Falling back to default: 5')
#
#            return ('', '', args, timeout)
#
#          except KeyError as exc:
#            rospy.logerr('SR2: Entry does not contain data that can be executed by the supported ROS tools. Add node, launch or service to YAML description')
#            raise exc
#    except KeyError as exc:
#      try:
#        # If finding "package" element inside the entry fails
#        pkg = ''
#        split_str = yamlEntry['app']
#        cmd_raw = split_str.split(' ', 1)
#        if len(cmd_raw) == 1: cmd = cmd_raw[0]
#        else:
#          cmd = cmd_raw[0]
#          args = cmd_raw[1]
#      except:
#        rospy.loginfo('SR2: Error while loading YAML file. Missing node, launch, service or app to YAML entry')
#        rospy.loginfo('SR2: "%s"' % exc)
#        return ('','','', 0)

#    return (pkg, cmd, args, 0)