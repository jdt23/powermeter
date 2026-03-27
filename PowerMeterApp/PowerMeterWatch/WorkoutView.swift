import SwiftUI

struct WorkoutView: View {
    @EnvironmentObject var workoutManager: WatchWorkoutManager
    @EnvironmentObject var connectivityManager: WatchConnectivityManager

    var body: some View {
        if workoutManager.isActive {
            TabView {
                mainScreen
                detailScreen
            }
            .tabViewStyle(.verticalPage)
            .ignoresSafeArea()
        } else {
            waitingView
        }
    }

    // MARK: - Waiting

    private var waitingView: some View {
        VStack(spacing: 8) {
            Image(systemName: "bicycle")
                .font(.system(size: 36))
                .foregroundColor(.green)
            Text("PowerMeter").font(.headline)
            if connectivityManager.isPhoneReachable {
                Text("Start workout\non iPhone")
                    .font(.caption2).foregroundColor(.gray).multilineTextAlignment(.center)
            } else {
                Text("Connecting to\niPhone...")
                    .font(.caption2).foregroundColor(.yellow).multilineTextAlignment(.center)
            }
        }
    }

    // MARK: - Main Screen

    private var mainScreen: some View {
        VStack(spacing: 3) {
            // Duration
            Text(fmtTime(workoutManager.elapsed))
                .font(.system(size: 18, weight: .bold, design: .monospaced))
                .foregroundColor(.white.opacity(0.8))

            // Power — hero
            HStack(alignment: .lastTextBaseline, spacing: 2) {
                Text("\(workoutManager.power)")
                    .font(.system(size: 48, weight: .heavy, design: .rounded))
                    .foregroundColor(.green)
                    .minimumScaleFactor(0.4).lineLimit(1)
                Text("W")
                    .font(.system(size: 14, weight: .semibold))
                    .foregroundColor(.green.opacity(0.5))
            }

            // HR | Cadence
            HStack(spacing: 3) {
                wTile(workoutManager.heartRate > 0 ? "\(Int(workoutManager.heartRate))" : "--", "BPM", .red)
                wTile("\(workoutManager.cadence)", "RPM", .cyan)
            }

            // Speed | Distance
            HStack(spacing: 3) {
                wTile(String(format: "%.1f", workoutManager.speed), "MPH", .mint)
                wTile(String(format: "%.2f", workoutManager.distance), "MI", .mint)
            }

            // Calories | Resistance
            HStack(spacing: 3) {
                wTile("\(Int(workoutManager.calories))", "CAL", .yellow)
                wTile("\(workoutManager.resistance)", "RES", .orange)
            }

            if workoutManager.isPaused {
                Text("PAUSED").font(.system(size: 11, weight: .bold)).foregroundColor(.yellow)
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(Color.black)
    }

    // MARK: - Detail Screen

    private var detailScreen: some View {
        VStack(spacing: 4) {
            Text("DETAILS").font(.system(size: 10, weight: .bold)).foregroundColor(.gray).tracking(1)
            dRow("Power", "\(workoutManager.power) W")
            dRow("Heart Rate", workoutManager.heartRate > 0 ? "\(Int(workoutManager.heartRate)) BPM" : "--")
            dRow("Cadence", "\(workoutManager.cadence) RPM")
            dRow("Resistance", "\(workoutManager.resistance)")
            dRow("Speed", String(format: "%.1f MPH", workoutManager.speed))
            dRow("Distance", String(format: "%.2f MI", workoutManager.distance))
            dRow("Calories", "\(Int(workoutManager.calories)) kcal")
            dRow("Duration", fmtTime(workoutManager.elapsed))
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(Color.black)
    }

    // MARK: - Components

    private func wTile(_ value: String, _ unit: String, _ color: Color) -> some View {
        VStack(spacing: 0) {
            Text(value)
                .font(.system(size: 24, weight: .bold, design: .rounded))
                .foregroundColor(.white)
                .minimumScaleFactor(0.4).lineLimit(1)
            Text(unit)
                .font(.system(size: 9, weight: .semibold))
                .foregroundColor(color.opacity(0.7))
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(RoundedRectangle(cornerRadius: 6).fill(Color.white.opacity(0.06)))
    }

    private func dRow(_ label: String, _ value: String) -> some View {
        HStack {
            Text(label).font(.system(size: 13)).foregroundColor(.gray)
            Spacer()
            Text(value).font(.system(size: 13, weight: .semibold)).foregroundColor(.white)
        }
    }

    private func fmtTime(_ t: TimeInterval) -> String {
        let h = Int(t) / 3600; let m = (Int(t) % 3600) / 60; let s = Int(t) % 60
        return h > 0 ? String(format: "%d:%02d:%02d", h, m, s) : String(format: "%02d:%02d", m, s)
    }
}
