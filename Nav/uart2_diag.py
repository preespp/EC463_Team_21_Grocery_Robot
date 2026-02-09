import serial, struct, time

PORT='/dev/ttyUSB0'
ser=serial.Serial(PORT,115200,timeout=0.01)

def cmd():
    p=struct.pack('<5fHBB',0,0,0,1,0,(1<<0)|(1<<4),1,1)
    return bytes([0xAC])+p+bytes([sum(p)&0xFF])

buf=bytearray()
tx=rx=ok=bad=hdr=0
t0=time.time()
last=time.time()

while time.time()-t0 < 8:
    ser.write(cmd()); tx += 1
    chunk=ser.read(512); rx += len(chunk); buf.extend(chunk)

    while True:
        i=buf.find(b'\xAB')
        if i < 0:
            break
        hdr += 1
        if len(buf)-i < 26:
            if i>0: del buf[:i]
            break
        fr=buf[i:i+26]; del buf[:i+26]
        data=fr[1:25]
        if (sum(data)&0xFF) != fr[25]:
            bad += 1
            continue
        ok += 1

    if time.time()-last > 1.0:
        print(f"tx={tx} rx_bytes={rx} hdr={hdr} ok={ok} bad={bad}")
        last=time.time()
    time.sleep(0.02)

ser.close()

