import time
from xbee_cs import xb_cs


def main():
    time.sleep(1)
    for i in range(13):
        suid = i + 1
        for j in range(suid):
            xb_cs.queue_cmd(50001, [suid])
            time.sleep(0.25)
            xb_cs.queue_cmd(50000, [suid])
            time.sleep(0.25)
        time.sleep(1)

    time.sleep(1)

    xb_cs.queue_cmd(50001, [0])
    time.sleep(0.25)
    xb_cs.queue_cmd(50000, [0])

    time.sleep(4)

    for i in range(13):
        xb_cs.queue_cmd(50001, [1])
        time.sleep(0.25)
        xb_cs.queue_cmd(50000, [1])
        time.sleep(0.25)


if __name__ == "__main__":
    main()
