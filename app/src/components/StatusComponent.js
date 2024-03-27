export default function StatusComponent({ text, staus, changeStatus = null }) {
  if (changeStatus !== null) {
    return (
      <button
        className={
          'status-component status-component-button ' +
          (staus ? 'status-green' : 'status-red')
        }
        onClick={changeStatus}
      >
        {text}
      </button>
    )
  }

  return (
    <div
      className={'status-component ' + (staus ? 'status-green' : 'status-red')}
    >
      {text}
    </div>
  )
}
