import Foundation
import CoreBluetooth
import Combine

enum BLEConnectionState: String {
    case disconnected = "Disconnected"
    case scanning = "Scanning..."
    case connecting = "Connecting..."
    case connected = "Connected"
}

class BLEManager: NSObject, ObservableObject {
    @Published var connectionState: BLEConnectionState = .disconnected
    @Published var power: UInt16 = 0
    @Published var cadence: UInt8 = 0
    @Published var resistance: UInt8 = 0

    private var centralManager: CBCentralManager!
    private var peripheral: CBPeripheral?

    // Must match sensor firmware UUIDs
    private let serviceUUID = CBUUID(string: "1818")
    private let powerUUID = CBUUID(string: "2A63")
    private let resistanceUUID = CBUUID(string: "2AD6")
    private let cadenceUUID = CBUUID(string: "2A5B")

    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }

    func startScanning() {
        guard centralManager.state == .poweredOn else { return }
        connectionState = .scanning
        // Allow duplicates so we keep getting advertisements even for already-seen peripherals
        centralManager.scanForPeripherals(
            withServices: [serviceUUID],
            options: [CBCentralManagerScanOptionAllowDuplicatesKey: false]
        )
    }

    func stopScanning() {
        centralManager.stopScan()
        if connectionState == .scanning {
            connectionState = .disconnected
        }
    }

    func disconnect() {
        if let peripheral = peripheral {
            centralManager.cancelPeripheralConnection(peripheral)
        }
    }
}

// MARK: - CBCentralManagerDelegate
extension BLEManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            startScanning()
        } else {
            connectionState = .disconnected
        }
    }

    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                         advertisementData: [String: Any], rssi RSSI: NSNumber) {
        // Connect to any peripheral advertising the Cycling Power Service (1818).
        // The sensor is the only device that will advertise this custom service.
        // Name may not be available in advertisement data on iOS, so don't filter by it.
        self.peripheral = peripheral
        centralManager.stopScan()
        connectionState = .connecting
        centralManager.connect(peripheral, options: [
            CBConnectPeripheralOptionNotifyOnDisconnectionKey: true
        ])
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        connectionState = .connected
        peripheral.delegate = self
        peripheral.discoverServices([serviceUUID])
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        self.peripheral = nil
        DispatchQueue.main.async {
            self.connectionState = .disconnected
            self.power = 0
            self.cadence = 0
            self.resistance = 0
        }
        // Auto-reconnect
        startScanning()
    }

    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        self.peripheral = nil
        connectionState = .disconnected
        startScanning()
    }
}

// MARK: - CBPeripheralDelegate
extension BLEManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let services = peripheral.services else { return }
        for service in services where service.uuid == serviceUUID {
            peripheral.discoverCharacteristics([powerUUID, resistanceUUID, cadenceUUID], for: service)
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard let characteristics = service.characteristics else { return }
        for characteristic in characteristics {
            if characteristic.properties.contains(.notify) {
                peripheral.setNotifyValue(true, for: characteristic)
            }
            if characteristic.properties.contains(.read) {
                peripheral.readValue(for: characteristic)
            }
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard let data = characteristic.value, error == nil else { return }

        DispatchQueue.main.async {
            switch characteristic.uuid {
            case self.powerUUID:
                // Standard Cycling Power Measurement: bytes 0-1 = flags, bytes 2-3 = power
                if data.count >= 4 {
                    self.power = UInt16(data[2]) | (UInt16(data[3]) << 8)
                } else if data.count >= 2 {
                    // Fallback for old firmware format (raw uint16)
                    self.power = data.withUnsafeBytes { $0.load(as: UInt16.self) }
                }
            case self.resistanceUUID:
                if data.count >= 1 {
                    self.resistance = data[0]
                }
            case self.cadenceUUID:
                if data.count >= 1 {
                    self.cadence = data[0]
                }
            default:
                break
            }
        }
    }
}
