import SwiftUI

struct SettingsView: View {
    @EnvironmentObject var workoutSession: WorkoutSession
    @Environment(\.dismiss) var dismiss

    @AppStorage("ftp") private var ftp: Double = 200.0
    @State private var ftpText: String = ""

    var body: some View {
        NavigationView {
            Form {
                Section(header: Text("Power Zones")) {
                    HStack {
                        Text("FTP (Functional Threshold Power)")
                        Spacer()
                        TextField("200", text: $ftpText)
                            .keyboardType(.numberPad)
                            .multilineTextAlignment(.trailing)
                            .frame(width: 60)
                        Text("W").foregroundColor(.gray)
                    }

                    VStack(alignment: .leading, spacing: 6) {
                        zoneRow(.z1, ftp: ftp)
                        zoneRow(.z2, ftp: ftp)
                        zoneRow(.z3, ftp: ftp)
                        zoneRow(.z4, ftp: ftp)
                        zoneRow(.z5, ftp: ftp)
                        zoneRow(.z6, ftp: ftp)
                    }
                    .padding(.vertical, 4)
                }

                Section(header: Text("About")) {
                    HStack {
                        Text("Sensor UUID")
                        Spacer()
                        Text("1818").foregroundColor(.gray)
                    }
                    HStack {
                        Text("Sample Rate")
                        Spacer()
                        Text("Every 2s").foregroundColor(.gray)
                    }
                }
            }
            .navigationTitle("Settings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .confirmationAction) {
                    Button("Done") {
                        if let val = Double(ftpText), val > 0 {
                            ftp = val
                            workoutSession.ftp = val
                        }
                        dismiss()
                    }
                }
            }
            .onAppear {
                ftpText = "\(Int(ftp))"
                workoutSession.ftp = ftp
            }
        }
    }

    private func zoneRow(_ zone: PowerZone, ftp: Double) -> some View {
        let ranges: [(PowerZone, String)] = [
            (.z1, "< \(Int(ftp * 0.55))W"),
            (.z2, "\(Int(ftp * 0.55))-\(Int(ftp * 0.75))W"),
            (.z3, "\(Int(ftp * 0.75))-\(Int(ftp * 0.90))W"),
            (.z4, "\(Int(ftp * 0.90))-\(Int(ftp * 1.05))W"),
            (.z5, "\(Int(ftp * 1.05))-\(Int(ftp * 1.20))W"),
            (.z6, "> \(Int(ftp * 1.20))W"),
        ]
        let range = ranges.first(where: { $0.0 == zone })?.1 ?? ""

        return HStack {
            Circle().fill(zoneColor(zone)).frame(width: 10, height: 10)
            Text("Z\(zone.rawValue)").font(.system(size: 13, weight: .bold))
            Text(zone.label).font(.system(size: 12)).foregroundColor(.gray)
            Spacer()
            Text(range).font(.system(size: 12, design: .monospaced)).foregroundColor(.secondary)
        }
    }

    private func zoneColor(_ zone: PowerZone) -> Color {
        switch zone {
        case .z1: return .gray
        case .z2: return .blue
        case .z3: return .green
        case .z4: return .yellow
        case .z5: return .orange
        case .z6: return .red
        }
    }
}
