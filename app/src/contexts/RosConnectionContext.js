import { useEffect, useState, useRef, useContext, createContext } from 'react'
import ROSLIB from 'roslib'
import { rosSettings } from '../settings/rosSettings'

const RosContext = createContext(null)

export function useRosHook() {
  return useContext(RosContext)
}

export function RosProvider({ children }) {
  return (
    <RosContext.Provider value={RosHookProvider()}>
      {children}
    </RosContext.Provider>
  )
}

function RosHookProvider() {
  const rosRef = useRef(null)
  const [rosStatus, setRosStatus] = useState({
    status: 'disconnected',
    message: 'Not connected to websocket server.',
  })

  useEffect(() => {
    let ros = new ROSLIB.Ros({
      url: rosSettings.url,
    })

    rosRef.current = ros

    ros.on('connection', () => {
      setRosStatus({
        status: 'connected',
        message: 'Connected to websocket server.',
      })

      console.log('Connected to websocket server.')
    })

    ros.on('error', (error) => {
      setRosStatus({
        status: 'error',
        message: 'Error connecting to websocket server: ' + error,
      })

      if (rosRef.current === ros) rosRef.current = null

      console.log('Error connecting to websocket server: ', error)
    })

    ros.on('close', () => {
      setRosStatus({
        status: 'disconnected',
        message: 'Connection to websocket server closed.',
      })

      if (rosRef.current === ros) rosRef.current = null

      console.log('Connection to websocket server closed.')
    })

    return () => {
      ros.close()
      rosRef.current = null
    }
  }, [])

  return {
    rosRef,
    rosStatus,
  }
}
