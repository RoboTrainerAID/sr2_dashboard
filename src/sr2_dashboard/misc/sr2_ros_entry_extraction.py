import rospy
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
    args = []  # Args is the actual ROS node/launch file/etc. we want to start

    #Try each of the possible configurations: node, launch and service
    try:
      pkg = yamlEntry['package']
      rospy.loginfo('Using package %s' % pkg)
      try:
        args = yamlEntry['node']
        rospy.loginfo('Nodes detected. Will use "rosrun"')
        cmd = 'rosrun'

      except KeyError:
        try:
          _args = yamlEntry['launch']
          rospy.loginfo('Launch file(s) detected. Will use "roslaunch"')
          cmd = 'roslaunch'

          args.append(_args + '.launch')
          rospy.loginfo('Single launch file detected: "%s"' % args)

        except KeyError:
          try:
            args.append(yamlEntry['serivce'])
            rospy.loginfo('Service deteceted. Will use "rosservice call"')
            cmd = 'rosservice call'

          except KeyError as exc:
            rospy.logerror('Button does not contain data that can be executed by the supported ROS tools. Add node, launch or service to the button"s description')
            raise exc
    except KeyError as exc:
      rospy.loginfo('SR2: error while loading YAML file.')
      rospy.loginfo('SR2: full message: \n"%s"' % exc)
      return ['','','']

    return (pkg, cmd, args)
