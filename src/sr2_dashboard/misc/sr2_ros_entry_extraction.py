# Author: Aleksandar Vladimirov Atanasov
# Description: Extracts data from menu_entry or button

import rospy

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
    icons_view = []
    res_icons = None
    converted_icons = []

    # Add icons
    if with_view:
      icons_view.append(['status/status_inactive.svg'])         # Inactive
      icons_view.append(['status/status_running.svg'])          # Active
      icons_view.append(['status/status_error.svg'])            # Failed
    else:
      icons.append(['control/menu/diagnostics_inactive.png'])   # Inactive view
      icons.append(['control/menu/diagnostics_running.png'])    # Active view
      icons.append(['control/menu/diagnostics_error.png'])      # Failed

    res_icons = list(icons_view if with_view else icons)
    converted_icons = icon_helper.set_icon_lists(icons_view if with_view else icons)

    converted_clicked_icons = []

    return (converted_icons[0], converted_clicked_icons[0])

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

    menu_entry:
      buttons:
        - captions:
          ...
          package: "lt"----------------|_____data source
          node: "talker.py"------------|
    '''
    assert yamlEntry != None, "Empty button configuration"
    pkg = ''
    cmd = ''   # Can be rosrun/roslaunch/rosservice call
    args = ''  # Args is the actual ROS node/launch file/etc. we want to start (exception: see "app" case)

    #Try each of the possible configurations: node, launch and service
    try:
      pkg = yamlEntry['package']
      rospy.loginfo('SR2: Using package %s' % pkg)
      try:
        args = yamlEntry['node']
        rospy.loginfo('SR2: Nodes detected. Will use "rosrun"')
        cmd = 'rosrun'

      except KeyError:
        try:
          args = yamlEntry['launch'] + '.launch'
          rospy.loginfo('SR2: Launch file detected. Will use "roslaunch"')
          cmd = 'roslaunch'

        except KeyError:
          try:
            args = yamlEntry['service']
            rospy.loginfo('SR2: Service deteceted. Will use "rosservice call"')
            cmd = 'rosservice call'
            timeout = 5
            if 'timeout' in yamlEntry:
              try:
                timeout = int(yamlEntry['timeout'])
              except:
                rospy.logerr('SR2: Timeout found however not an integer. Falling back to default: 5')

            return (pkg, cmd, args, timeout)
          except KeyError as exc:
            rospy.logerr('SR2: Entry does not contain data that can be executed by the supported ROS tools. Add node, launch or service to YAML description')
            raise exc


    except KeyError as exc:
      try:
        # If finding "package" element inside the entry fails
        pkg = ''
        split_str = yamlEntry['app']
        cmd_raw = split_str.split(' ', 1)
        if len(cmd_raw) == 1: cmd = cmd_raw[0]
        else:
          cmd = cmd_raw[0]
          args = cmd_raw[1]
      except:
        rospy.loginfo('SR2: error while loading YAML file. Missing node, launch, service or app to YAML entry')
        rospy.loginfo('SR2: full message: \n"%s"' % exc)
        return ['','','']

    return (pkg, cmd, args)
