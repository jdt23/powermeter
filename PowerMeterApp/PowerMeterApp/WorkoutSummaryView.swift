import SwiftUI

struct WorkoutSummaryView: View {
    @EnvironmentObject var healthKitManager: HealthKitManager
    @EnvironmentObject var workoutSession: WorkoutSession

    let summary: WorkoutSummary
    let onDismiss: () -> Void

    @State private var saveState: SaveState = .unsaved

    enum SaveState { case unsaved, saving, saved, failed, watchSaved }

    var body: some View {
        VStack(spacing: 8) {
            Text("Workout Complete")
                .font(.title3).fontWeight(.bold).foregroundColor(.white)

            LazyVGrid(columns: [GridItem(.flexible()), GridItem(.flexible()), GridItem(.flexible())], spacing: 6) {
                stat("Duration", formatDuration(summary.duration), .white)
                stat("Distance", String(format: "%.2f mi", summary.totalDistance), .mint)
                stat("Calories", "\(Int(summary.totalCalories))", .yellow)
                stat("Avg Power", "\(Int(summary.averagePower)) W", .green)
                stat("Max Power", "\(Int(summary.maxPower)) W", .green)
                stat("NP", "\(Int(summary.normalizedPower)) W", .purple)
                stat("IF", String(format: "%.2f", summary.intensityFactor), .indigo)
                stat("TSS", "\(Int(summary.tss))", .pink)
                stat("FTP", "\(Int(summary.ftp)) W", .gray)
                stat("Avg Cadence", "\(Int(summary.averageCadence))", .cyan)
                stat("Max Cadence", "\(Int(summary.maxCadence))", .cyan)
                stat("Avg Speed", String(format: "%.1f mph", summary.averageSpeed), .mint)
                stat("Avg Resist.", "\(Int(summary.averageResistance))", .orange)
                stat("Max Resist.", "\(Int(summary.maxResistance))", .orange)
                stat("Max Speed", String(format: "%.1f mph", summary.maxSpeed), .mint)
                if summary.averageHeartRate > 0 {
                    stat("Avg HR", "\(Int(summary.averageHeartRate))", .red)
                    stat("Max HR", "\(Int(summary.maxHeartRate))", .red)
                }
            }
            .padding(.horizontal, 12)

            // Health save — skip if Watch already saved
            VStack(spacing: 4) {
                switch saveState {
                case .watchSaved:
                    Label("Saved by Apple Watch", systemImage: "applewatch.checkmark")
                        .font(.caption).foregroundColor(.green).padding(.vertical, 6)
                case .unsaved:
                    Button(action: saveToHealth) {
                        Label("Save to Apple Health", systemImage: "heart.fill")
                            .font(.system(size: 14, weight: .bold))
                            .frame(maxWidth: .infinity).padding(.vertical, 10)
                            .background(Color.red).foregroundColor(.white).cornerRadius(8)
                    }
                case .saving:
                    HStack(spacing: 6) {
                        ProgressView().tint(.white)
                        Text("Saving...").font(.caption).foregroundColor(.gray)
                    }.padding(.vertical, 6)
                case .saved:
                    Label("Saved to Apple Health", systemImage: "checkmark.circle.fill")
                        .font(.caption).foregroundColor(.green).padding(.vertical, 6)
                case .failed:
                    VStack(spacing: 3) {
                        Label("Failed", systemImage: "exclamationmark.circle.fill")
                            .font(.caption).foregroundColor(.orange)
                        Button("Retry", action: saveToHealth)
                            .font(.caption).padding(.horizontal, 16).padding(.vertical, 5)
                            .background(Color.red).foregroundColor(.white).cornerRadius(6)
                    }
                }
            }
            .padding(.horizontal, 12)

            Button(action: onDismiss) {
                Text("Done").font(.system(size: 15, weight: .bold))
                    .frame(maxWidth: .infinity).padding(.vertical, 12)
                    .background(Color.blue).foregroundColor(.white).cornerRadius(10)
            }
            .padding(.horizontal, 12).padding(.bottom, 8)
        }
        .onAppear {
            // If Watch already saved the workout, don't save again
            if summary.watchSaved { saveState = .watchSaved }
        }
    }

    private func stat(_ label: String, _ value: String, _ color: Color) -> some View {
        VStack(spacing: 1) {
            Text(label).font(.system(size: 9, weight: .medium)).foregroundColor(color.opacity(0.7))
                .lineLimit(1).minimumScaleFactor(0.7)
            Text(value).font(.system(size: 16, weight: .bold, design: .rounded)).foregroundColor(.white)
                .lineLimit(1).minimumScaleFactor(0.6)
        }
        .frame(maxWidth: .infinity).padding(.vertical, 8)
        .background(RoundedRectangle(cornerRadius: 8).fill(Color.white.opacity(0.04)))
    }

    private func saveToHealth() {
        saveState = .saving
        healthKitManager.saveWorkout(
            start: summary.startDate, end: summary.endDate,
            powerSamples: workoutSession.powerSamples,
            cadenceSamples: workoutSession.cadenceSamples,
            heartRateSamples: workoutSession.heartRateSamples,
            totalCalories: summary.totalCalories
        ) { success, _ in
            DispatchQueue.main.async { saveState = success ? .saved : .failed }
        }
    }

    private func formatDuration(_ interval: TimeInterval) -> String {
        let h = Int(interval) / 3600; let m = (Int(interval) % 3600) / 60; let s = Int(interval) % 60
        return h > 0 ? String(format: "%d:%02d:%02d", h, m, s) : String(format: "%02d:%02d", m, s)
    }
}
