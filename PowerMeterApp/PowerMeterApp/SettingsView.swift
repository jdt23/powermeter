import SwiftUI

struct SettingsView: View {
    @EnvironmentObject var workoutSession: WorkoutSession
    @Environment(\.dismiss) var dismiss

    @AppStorage("ftp") private var ftp: Double = 200.0
    @AppStorage("maxHR") private var maxHR: Double = 190.0
    @State private var ftpText: String = ""
    @State private var maxHRText: String = ""

    var body: some View {
        NavigationView {
            Form {
                Section(header: Text("Heart Rate Zones")) {
                    HStack {
                        Text("Max Heart Rate")
                        Spacer()
                        TextField("190", text: $maxHRText)
                            .keyboardType(.numberPad).multilineTextAlignment(.trailing).frame(width: 60)
                        Text("BPM").foregroundColor(.gray)
                    }
                    VStack(alignment: .leading, spacing: 6) {
                        hrZoneRow(.z1, maxHR: maxHR)
                        hrZoneRow(.z2, maxHR: maxHR)
                        hrZoneRow(.z3, maxHR: maxHR)
                        hrZoneRow(.z4, maxHR: maxHR)
                        hrZoneRow(.z5, maxHR: maxHR)
                    }.padding(.vertical, 4)
                }

                Section(header: Text("Power")) {
                    HStack {
                        Text("FTP")
                        Spacer()
                        TextField("200", text: $ftpText)
                            .keyboardType(.numberPad).multilineTextAlignment(.trailing).frame(width: 60)
                        Text("W").foregroundColor(.gray)
                    }
                }

                Section(header: Text("Power Saving")) {
                    Text("Reduce BLE scan interval when connected").font(.caption).foregroundColor(.gray)
                    Text("Screen dims after 30s of inactivity during pause").font(.caption).foregroundColor(.gray)
                    Text("Watch HR streaming stops when phone HR available").font(.caption).foregroundColor(.gray)
                }

                Section(header: Text("About")) {
                    HStack { Text("Sensor UUID"); Spacer(); Text("1818").foregroundColor(.gray) }
                    HStack { Text("Sample Rate"); Spacer(); Text("Every 2s").foregroundColor(.gray) }
                }
            }
            .navigationTitle("Settings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .confirmationAction) {
                    Button("Done") {
                        if let val = Double(ftpText), val > 0 { ftp = val; workoutSession.ftp = val }
                        if let val = Double(maxHRText), val > 0 { maxHR = val; workoutSession.maxHR = val }
                        dismiss()
                    }
                }
            }
            .onAppear { ftpText = "\(Int(ftp))"; maxHRText = "\(Int(maxHR))" }
        }
    }

    private func hrZoneRow(_ zone: HeartRateZone, maxHR: Double) -> some View {
        let ranges: [(HeartRateZone, String)] = [
            (.z1, "< \(Int(maxHR * 0.60)) BPM"),
            (.z2, "\(Int(maxHR * 0.60))-\(Int(maxHR * 0.70))"),
            (.z3, "\(Int(maxHR * 0.70))-\(Int(maxHR * 0.80))"),
            (.z4, "\(Int(maxHR * 0.80))-\(Int(maxHR * 0.90))"),
            (.z5, "> \(Int(maxHR * 0.90)) BPM"),
        ]
        let range = ranges.first(where: { $0.0 == zone })?.1 ?? ""
        let color: Color = { switch zone { case .z1: return .gray; case .z2: return .blue; case .z3: return .green; case .z4: return .orange; case .z5: return .red } }()

        return HStack {
            Circle().fill(color).frame(width: 10, height: 10)
            Text("Z\(zone.rawValue)").font(.system(size: 13, weight: .bold))
            Text(zone.label).font(.system(size: 12)).foregroundColor(.gray)
            Spacer()
            Text(range).font(.system(size: 12, design: .monospaced)).foregroundColor(.secondary)
        }
    }
}
