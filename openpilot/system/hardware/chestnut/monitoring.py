import struct

import usb1

import openpilot.cereal.messaging as messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, get_usb_state, is_chestnut_usb_id


def read_chestnut_state(handle, gpu_state=None):
  msg = messaging.new_message('chestnutState')
  if gpu_state is not None:
    msg.chestnutState = gpu_state
  state = msg.chestnutState
  try:
    raw = handle.controlRead(0xC0, 0xC0, 0, 0, 5, timeout=100)
    state.supplyVoltage, state.supplyCurrent, state.supplyFault = struct.unpack('<Hh?', bytes(raw))
    raw = handle.controlRead(0xC0, 0xE4, 0xB450, 0, 1, timeout=100)
    state.pcieLtssm, = struct.unpack('B', bytes(raw))
    msg.valid = True
  except (usb1.USBError, struct.error):
    msg.valid = False
  return msg


def chestnut_state_thread(end_event, exclusive_flag=None):
  # [CHX_USB_EXCLUSIVE_20261101] exclusive_flag 为 dict 且 'on' 为 True 时
  # （点火/行车）主动 close 并暂停，把 eGPU 独占让给 modeld。否则
  # modeld 侧 tinygrad 会拿到 libusb_set_configuration: Resource busy。
  pm = messaging.PubMaster(['chestnutState'])
  sm = messaging.SubMaster(['chestnutGpuState'])
  with usb1.USBContext() as context:
    handle = None
    try:
      while not end_event.is_set():
        # [CHX_USB_EXCLUSIVE_20261101] 行车时让出设备
        if exclusive_flag is not None and exclusive_flag.get('on'):
          if handle is not None:
            try:
              handle.close()
            except Exception:
              pass
            handle = None
          end_event.wait(1.0)
          continue
        if handle is None:
          devices = [d for d in get_usb_state() if is_chestnut_usb_id(d['vendorId'], d['productId']) and
                     d['product'] == CHESTNUT_USB_PRODUCT]
          if len(devices) == 1:
            try:
              handle = context.openByVendorIDAndProductID(devices[0]['vendorId'], devices[0]['productId'], skip_on_error=True)
            except usb1.USBError:
              pass
        if handle is not None:
          sm.update(0)
          gpu_valid = sm.alive['chestnutGpuState'] and sm.valid['chestnutGpuState']
          msg = read_chestnut_state(handle, sm['chestnutGpuState'] if gpu_valid else None)
          if not msg.valid:
            handle.close()
            handle = None
          msg.valid &= not sm.seen['chestnutGpuState'] or gpu_valid
          pm.send('chestnutState', msg)
        end_event.wait(1 / SERVICE_LIST['chestnutState'].frequency if handle is not None else 1.)
    finally:
      if handle is not None:
        handle.close()
