const rosSettings = {
  url: 'ws://localhost:9090',
}

const motorSettings = {
  maxStrength: 100,
  minStrength: 13,
  getMotorJointsLenghtService: {
    name: '/virtual_dc_motor_node/get_joints_length',
    serviceType: 'virtual_dc_motor/getMotorJointsLength',
  },
  0: {
    positionTopic: {
      name: '/virtual_dc_motor_node/get_position_0',
      messageType: 'std_msgs/UInt16',
    },
    controlTopic: {
      name: '/virtual_dc_motor_node/set_cs_0',
      messageType: 'std_msgs/Int8',
    },
  },
  1: {
    positionTopic: {
      name: '/virtual_dc_motor_node/get_position_1',
      messageType: 'std_msgs/UInt16',
    },
    controlTopic: {
      name: '/virtual_dc_motor_node/set_cs_1',
      messageType: 'std_msgs/Int8',
    },
  },
  2: {
    positionTopic: {
      name: '/virtual_dc_motor_node/get_position_2',
      messageType: 'std_msgs/UInt16',
    },
    controlTopic: {
      name: '/virtual_dc_motor_node/set_cs_2',
      messageType: 'std_msgs/Int8',
    },
  },
}

export { rosSettings, motorSettings }
