import argparse
import math
import m5
from m5.objects import *
from m5.util import *

import sys

sys.path.append("/home/qyzhang/gem5/configs")

from common import Options
from common.Caches import *

res_path = "/home/qyzhang/SimpleSSD-FullSystem/m5/"
disk_path = res_path + "disks/linaro-aarch64-linux.img"
bootloader = res_path + "binaries/boot.arm64"
kernel = res_path + "binaries/aarch64-vmlinux-4.9.92"
dtb_file = res_path + "binaries/armv8_gem5_v1_4cpu.dtb"
mem_size = "4GB"

#### Command Line Arguments
parser = argparse.ArgumentParser()
Options.addCommonOptions(parser)
Options.addFSOptions(parser)
parser.add_argument(
    "--ssd-interface",
    action="store",
    type=str,
    default="nvme",
    help="Interface to use to connect SimpleSSD.",
)
parser.add_argument(
    "--ssd-config",
    action="store",
    type=str,
    default=None,
    help="Path to SimpleSSD configuration file.",
)
parser.add_argument(
    "--disable-ide",
    action="store_true",
    help="Disable default IDE controller.",
)
args = parser.parse_args()

np = args.num_cpus
simplessd = {
    "interface": "nvme",
    "config": "/home/qyzhang/gem5/src/dev/storage/simplessd/config/sample.cfg",
    "disable_ide": False,
}


system = ArmSystem()
pci_devices = []


class MemBus(SystemXBar):
    badaddr_responder = BadAddr()
    default = Self.badaddr_responder.pio


class IOCache(Cache):
    assoc = 8
    tag_latency = 50
    data_latency = 50
    response_latency = 50
    mshrs = 20
    size = "1kB"
    tgts_per_mshr = 12


system.membus = MemBus()
system.membus.badaddr_responder.warn_access = "warn"

system.iobus = IOXBar()

# SimpleSSD
system.bridge = Bridge(delay="50ns")

system.bridge.mem_side_port = system.iobus.cpu_side_ports
system.bridge.cpu_side_port = system.membus.mem_side_ports

system.realview = VExpress_GEM5_V1()
system._bootmem = system.realview.bootmem

# Attach any PCI devices this platform supports
system.realview.attachPciDevices()


class CowIdeDisk(IdeDisk):
    image = CowDiskImage(child=RawDiskImage(read_only=True), read_only=False)

    def childImage(self, ci):
        self.image.child.image_file = ci


def makeCowDisks(disk_paths):
    disks = []
    for disk_path in disk_paths:
        disk = CowIdeDisk(driveID="device0")
        disk.childImage(disk_path)
        disks.append(disk)
    return disks


disks = makeCowDisks([disk_path])

system.pci_ide = IdeController(disks=disks)
pci_devices.append(system.pci_ide)

if simplessd["interface"] == "nvme":
    system.pci_nvme = NVMeInterface(SSDConfig=simplessd["config"])
    pci_devices.append(system.pci_nvme)
elif simplessd["interface"] == "ufs":
    system.realview.ufs.SSDConfig = simplessd["config"]
elif simplessd["interface"] == "ocssd" or simplessd["interface"] == "ocssd1":
    system.pci_nvme = OCSSDInterface12(SSDConfig=simplessd["config"])
    pci_devices.append(system.pci_nvme)
elif simplessd["interface"] == "ocssd2":
    system.pci_nvme = OCSSDInterface20(SSDConfig=simplessd["config"])
    pci_devices.append(system.pci_nvme)
elif simplessd["interface"] == "sata":
    system.pci_sata = SATAInterface(SSDConfig=simplessd["config"])
    pci_devices.append(system.pci_sata)
else:
    fatal("Undefined SimpleSSD interface {}!".format(simplessd["interface"]))

system.mem_ranges = []
size_remain = int(Addr(mem_size))
for region in system.realview._mem_regions:
    if size_remain > int(region.size()):
        system.mem_ranges.append(region)
        size_remain = size_remain - int(region.size())
    else:
        system.mem_ranges.append(AddrRange(region.start, size=size_remain))
        size_remain = 0
        break
    warn(
        "Memory size specified spans more than one region. Creating"
        " another memory controller for that range."
    )

if size_remain > 0:
    fatal(
        "The currently selected ARM platforms doesn't support"
        " the amount of DRAM you've selected. Please try"
        " another platform"
    )

# Ensure that writes to the UART actually go out early in the boot
cmdline = (
    "earlyprintk=pl011,0x1c090000 console=ttyAMA0 "
    + "lpj=19988480 norandmaps rw loglevel=8 "
    + f"mem={mem_size} root=/dev/sda1 "
    + "kpti=0 nospectre_v2 ssbd=force-off"  # SimpleSSD
)

workload = ArmFsLinux()
workload.dtb_filename = dtb_file
workload.machine_type = (
    "VExpress_GEM5_V1"
    if "VExpress_GEM5_V1" in ArmMachineType.map
    else "DTOnly"
)
workload.command_line = cmdline
workload.object_file = kernel
system.workload = workload

if hasattr(system.realview.gic, "cpu_addr"):
    system.gic_cpu_addr = system.realview.gic.cpu_addr

system.realview.setupBootLoader(system, None, boot_loader=bootloader)
system.realview.attachOnChipIO(system.membus, system.bridge)

# Attach off-chip devices
system.realview.attachIO(system.iobus)
for dev in pci_devices:
    system.realview.attachPciDevice(dev, system.iobus, dma_ports=None)

system.realview.pci_host.pci_mem_base = 0

system.terminal = Terminal()
system.vncserver = VncServer()

system.system_port = system.membus.cpu_side_ports

# Create a top-level voltage domain
system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)
# Create a source clock for the system and set the clock period
system.clk_domain = SrcClockDomain(
    clock=args.sys_clock, voltage_domain=system.voltage_domain
)
# Create a CPU voltage domain
system.cpu_voltage_domain = VoltageDomain()
# Create a source clock for the CPUs and set the clock period
system.cpu_clk_domain = SrcClockDomain(
    clock=args.cpu_clock, voltage_domain=system.cpu_voltage_domain
)

if args.script is not None:
    system.readfile = args.script

system.init_param = args.init_param

# For now, assign all the CPUs to the same clock domain
system.cpu = [
    ArmAtomicSimpleCPU(clk_domain=system.cpu_clk_domain, cpu_id=i)
    for i in range(np)
]

# By default the IOCache runs at the system clock
system.iocache = IOCache(addr_ranges=system.mem_ranges)
system.iocache.cpu_side = system.iobus.mem_side_ports
system.iocache.mem_side = system.membus.cpu_side_ports

gicv2m_range = AddrRange(0x2c1c0000, 0x2c1d0000 - 1)
system.iobridge = Bridge(delay="50ns", ranges=[gicv2m_range])
system.iobridge.cpu_side_port = system.iobus.mem_side_ports
system.iobridge.mem_side_port = system.membus.cpu_side_ports

for i in range(np):
    if args.simpoint_profile:
        system.cpu[i].addSimPointProbe(args.simpoint_interval)
    if args.checker:
        system.cpu[i].addCheckerCpu()
    system.cpu[i].createThreads()


#### Setup Cache
system.cache_line_size = args.cacheline_size


def _get_cache_opts(level, options):
    opts = {}

    size_attr = "{}_size".format(level)
    if hasattr(options, size_attr):
        opts["size"] = getattr(options, size_attr)

    assoc_attr = "{}_assoc".format(level)
    if hasattr(options, assoc_attr):
        opts["assoc"] = getattr(options, assoc_attr)

    return opts


if args.l2cache:
    # Provide a clock for the L2 and the L1-to-L2 bus here as they
    # are not connected using addTwoLevelCacheHierarchy. Use the
    # same clock as the CPUs.
    system.l2 = L2Cache(
        clk_domain=system.cpu_clk_domain, **_get_cache_opts("l2", args)
    )

    system.tol2bus = L2XBar(clk_domain=system.cpu_clk_domain)
    system.l2.cpu_side = system.tol2bus.mem_side_ports
    system.l2.mem_side = system.membus.cpu_side_ports

for i in range(np):
    if args.caches:
        icache = L1_ICache(**_get_cache_opts("l1i", args))
        dcache = L1_DCache(**_get_cache_opts("l1d", args))

        iwalkcache = None
        dwalkcache = None

        # When connecting the caches, the clock is also inherited
        # from the CPU in question
        system.cpu[i].addPrivateSplitL1Caches(
            icache, dcache, iwalkcache, dwalkcache
        )

    system.cpu[i].createInterruptController()
    system.cpu[i].connectAllPorts(
        system.tol2bus.cpu_side_ports,
        system.membus.cpu_side_ports,
        system.membus.mem_side_ports,
    )


#### Setup Memory

# Mandatory args
opt_mem_channels = args.mem_channels

# Optional args
opt_mem_channels_intlv = getattr(args, "mem_channels_intlv", 128)

xbar = system.membus

nbr_mem_ctrls = opt_mem_channels

intlv_bits = int(math.log(nbr_mem_ctrls, 2))
if 2**intlv_bits != nbr_mem_ctrls:
    fatal("Number of memory channels must be a power of 2")

mem_ctrls = []

# The default behaviour is to interleave memory channels on 128
# byte granularity, or cache line granularity if larger than 128
# byte. This value is based on the locality seen across a large
# range of workloads.
intlv_size = max(opt_mem_channels_intlv, system.cache_line_size.value)
intlv_low_bit = int(math.log(intlv_size, 2))

# For every range (most systems will only have one), create an
# array of memory interfaces and set their parameters to match
# their address mapping in the case of a DRAM
for r in system.mem_ranges:
    for i in range(nbr_mem_ctrls):
        # Create the DRAM interface
        dram_intf = DDR4_2400_8x8()

        # We got all we need to configure the appropriate address
        # range
        dram_intf.range = AddrRange(
            r.start,
            size=r.size(),
            intlvHighBit=intlv_low_bit + intlv_bits - 1,
            xorHighBit=0,
            intlvBits=intlv_bits,
            intlvMatch=i,
        )
        dram_intf.enable_dram_powerdown = None

        # Create the controller that will drive the interface
        mem_ctrl = dram_intf.controller()
        mem_ctrls.append(mem_ctrl)

# Connect the controller to the xbar port
for i in range(len(mem_ctrls)):
    # Connect the controllers to the membus
    mem_ctrls[i].port = xbar.mem_side_ports

system.mem_ctrls = mem_ctrls


#### Setup root and run simulation
root = Root(full_system=True, system=system)

# Setup global stat filtering.
stat_root_simobjs = []
for stat_root_str in args.stats_root:
    stat_root_simobjs.extend(root.get_simobj(stat_root_str))
m5.stats.global_dump_roots = stat_root_simobjs

root.apply_config(args.param)
m5.instantiate()

print("**** REAL SIMULATION ****")
exit_event = m5.simulate()
exit_cause = exit_event.getCause()
print(f"Exiting @ tick {m5.curTick()} because {exit_cause}")

if exit_event.getCode() != 0:
    print("Simulated exit code not 0! Exit code is", exit_event.getCode())
