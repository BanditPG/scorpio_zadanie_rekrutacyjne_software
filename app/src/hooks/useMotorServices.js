import ROSLIB from 'roslib'
import { useRosHook } from '../contexts/RosConnectionContext'
import { motorSettings } from '../settings/rosSettings'

export default function useMotorServices() {
  const { rosRef, rosStatus } = useRosHook()

  function getMotorJointsLenght() {
    if (rosStatus.status !== 'connected')
      return Promise.reject('ROS not connected')

    const service = new ROSLIB.Service({
      ros: rosRef.current,
      name: motorSettings.getMotorJointsLenghtService.name,
      serviceType: motorSettings.getMotorJointsLenghtService.serviceType,
    })

    return new Promise((resolve, reject) => {
      service.callService({}, result => {
        resolve(result.data)
      })
    })
  }

  return {
    getMotorJointsLenght,
  }
}
