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
        } else {
            waitingView
        }
    }

    // MARK: - Waiting

    private var waitingView: some View {
        VStack(spacing: 8) {
            Image(systemName: "bicycle")
                .font(.system(size: 32))
                .foregroundColor(.green)
            Text("PowerMeter")
                .font(.headline)
            if connectivityManager.isPhoneReachable {
                Text("Start workout\non iPhone")
                    .font(.caption2).foregroundColor(.gray).multilineTextAlignment(.center)
            } else {
                Text("Connecting to\niPhone...")
                    .font(.caption2).foregroundColor(.yellow).multilineTextAlignment(.center)
            }
        }
    }

    // MARK: - Main Screen (fits on one watch screen)

    private var mainScreen: some View {
        VStack(spacing: 2) {
            // Duration
            Text(fmtTime(workoutManager.elapsed))
                .font(.system(size: 16, weight: .bold, design: .monospaced))
                .foregroundColor(.white)

            // Power — big
            HStack(alignment: .lastTextBaseline, spacing: 2) {
                Text("\(workoutManager.power)")
                    .font(.system(size: 36, weight: .heavy, design: .rounded))
                    .foregroundColor(.green)
                    .minimumScaleFactor(0.5).lineLimit(1)
                Text("W")
                    .font(.system(size: 11, weight: .medium))
                    .foregroundColor(.green.opacity(0.6))
            }

            // HR | Cadence — side by side
            HStack(spacing: 4) {
                miniTile(
                    workoutManager.heartRate > 0 ? "\(Int(workoutManager.heartRate))" : "--",
                    "BPM", .red)
                miniTile("\(workoutManager.cadence)", "RPM", .cyan)
            }

            // Speed | Distance — side by side
            HStack(spacing: 4) {
                miniTile(String(format: "%.1f", workoutManager.speed), "MPH", .mint)
                miniTile(String(format: "%.2f", workoutManager.distance), "MI", .mint)
            }

            // Calories | Resistance — side by side
            HStack(spacing: 4) {
                miniTile("\(Int(workoutManager.calories))", "CAL", .yellow)
                miniTile("\(workoutManager.resistance)", "RES", .orange)
            }

            if workoutManager.isPaused {
                Text("PAUSED").font(.system(size: 10, weight: .bold)).foregroundColor(.yellow)
            }
        }
        .padding(.horizontal, 2)
    }

    // MARK: - Detail Screen (swipe down)

    private var detailScreen: some View {
        VStack(spacing: 6) {
            Text("DETAILS").font(.system(size: 10, weight: .bold)).foregroundColor(.gray).tracking(1)

            VStack(spacing: 4) {
                detailRow("Power", "\(workoutManager.power) W")
                detailRow("Heart Rate", workoutManager.heartRate > 0 ? "\(Int(workoutManager.heartRate)) BPM" : "--")
                detailRow("Cadence", "\(workoutManager.cadence) RPM")
                detailRow("Resistance", "\(workoutManager.resistance)")
                detailRow("Speed", String(format: "%.1f MPH", workoutManager.speed))
                detailRow("Distance", String(format: "%.2f MI", workoutManager.distance))
                detailRow("Calories", "\(Int(workoutManager.calories)) kcal")
                detailRow("Duration", fmtTime(workoutManager.elapsed))
            }
        }
        .padding(.horizontal, 4)
    }

    // MARK: - Components

    private func miniTile(_ value: String, _ unit: String, _ color: Color) -> some View {
        VStack(spacing: 0) {
            Text(value)
                .font(.system(size: 18, weight: .bold, design: .rounded))
                .foregroundColor(.white)
                .minimumScaleFactor(0.5).lineLimit(1)
            Text(unit)
                .font(.system(size: 8, weight: .medium))
                .foregroundColor(color.opacity(0.7))
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 2)
        .background(RoundedRectangle(cornerRadius: 6).fill(Color.white.opacity(0.06)))
    }

    private func detailRow(_ label: String, _ value: String) -> some View {
        HStack {
            Text(label).font(.system(size: 12)).foregroundColor(.gray)
            Spacer()
            Text(value).font(.system(size: 12, weight: .semibold)).foregroundColor(.white)
        }
    }

    private func fmtTime(_ t: TimeInterval) -> String {
        let h = Int(t) / 3600; let m = (Int(t) % 3600) / 60; let s = Int(t) % 60
        return h > 0 ? String(format: "%d:%02d:%02d", h, m, s) : String(format: "%02d:%02d", m, s)
    }
}
