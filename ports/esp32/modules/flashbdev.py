import esp

class PartitionBdev:
    def __init__(self, partition, sec_size):
        self.SEC_SIZE = sec_size
        self.label = partition['label']
        self.encrypted = partition['encrypted']
        self.handle = partition['handle']
        self.blocks = partition['size'] // sec_size

    def readblocks(self, n, buf):
        # print("part:readblocks(%s, %x(%d))" % (n, id(buf), len(buf)))
        esp.partition_read(self.handle, n * self.SEC_SIZE, buf)

    def writeblocks(self, n, buf):
        # print("part:writeblocks(%s, %x(%d))" % (n, id(buf), len(buf)))
        n *= self.SEC_SIZE
        esp.partition_erase_range(self.handle, n, len(buf))
        esp.partition_write(self.handle, n, buf)

    def ioctl(self, op, arg):
        # print("part:ioctl(%d, %r)" % (op, arg))
        if op == 4:  # BP_IOCTL_SEC_COUNT
            return self.blocks
        if op == 5:  # BP_IOCTL_SEC_SIZE
            return self.SEC_SIZE

class FlashBdev:
    def __init__(self, start, size, sec_size):
        self.blocks = (esp.flash_size() - start) // sec_size
        self.START_SEC = start // sec_size
        self.SEC_SIZE = sec_size

    def readblocks(self, n, buf):
        # print("readblocks(%s, %x(%d))" % (n, id(buf), len(buf)))
        esp.flash_read((n + self.START_SEC) * self.SEC_SIZE, buf)

    def writeblocks(self, n, buf):
        # print("writeblocks(%s, %x(%d))" % (n, id(buf), len(buf)))
        # assert len(buf) <= self.SEC_SIZE, len(buf)
        esp.flash_erase(n + self.START_SEC)
        esp.flash_write((n + self.START_SEC) * self.SEC_SIZE, buf)

    def ioctl(self, op, arg):
        # print("ioctl(%d, %r)" % (op, arg))
        if op == 4:  # BP_IOCTL_SEC_COUNT
            return self.blocks
        if op == 5:  # BP_IOCTL_SEC_SIZE
            return self.SEC_SIZE

size = esp.flash_size()
vfs = esp.partition_find_first(esp.PARTITION_TYPE_DATA, esp.PARTITION_SUBTYPE_DATA_FAT, "vfs")
if size < 1024*1024:
    # flash too small for a filesystem
    bdev = None
elif vfs is not None:
    bdev = PartitionBdev(vfs, esp.SPI_FLASH_SEC_SIZE)
else:
    # for now we use a fixed size for the filesystem
    bdev = FlashBdev(esp.flash_user_start(), size, esp.SPI_FLASH_SEC_SIZE)
