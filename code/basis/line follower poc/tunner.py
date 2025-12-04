import json
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal, optimize
from scipy.fft import fft, fftfreq
from collections import defaultdict
import warnings
warnings.filterwarnings('ignore')

def parse_log_file(filename):
    """Parse log.txt and extract relevant data."""
    data = []
    with open(filename, 'r') as f:
        for line in f:
            try:
                parts = line.strip().split('] ', 1)
                if len(parts) == 2:
                    timestamp = parts[0][1:]
                    json_str = parts[1].replace("'", '"')
                    entry = json.loads(json_str)
                    
                    if entry.get('running') == 1:
                        data.append(entry)
            except (json.JSONDecodeError, ValueError) as e:
                continue
    return data

def calculate_iae(errors):
    """Integral of Absolute Error - lower is better."""
    return np.sum(np.abs(errors))

def calculate_ise(errors):
    """Integral of Squared Error - penalizes large errors."""
    return np.sum(np.square(errors))

def calculate_itae(errors, time):
    """Integral of Time-weighted Absolute Error - penalizes persistent errors."""
    return np.sum(time * np.abs(errors))

def detect_oscillations(errors, timestamps):
    """Advanced oscillation detection using frequency analysis."""
    if len(errors) < 10:
        return 0, 0, False
    
    # Detrend the signal
    detrended = signal.detrend(errors)
    
    # Apply FFT
    N = len(detrended)
    dt = np.mean(np.diff(timestamps)) / 1000.0  # Convert to seconds
    
    if dt == 0:
        return 0, 0, False
    
    yf = fft(detrended)
    xf = fftfreq(N, dt)[:N//2]
    power = 2.0/N * np.abs(yf[:N//2])
    
    # Find dominant frequency
    if len(power) > 1:
        dominant_idx = np.argmax(power[1:]) + 1  # Skip DC component
        dominant_freq = xf[dominant_idx]
        dominant_power = power[dominant_idx]
        
        # Check if oscillation is significant
        is_oscillating = dominant_power > 0.05 and dominant_freq > 0.1
        
        return dominant_freq, dominant_power, is_oscillating
    
    return 0, 0, False

def calculate_settling_time(errors, timestamps, threshold=0.02):
    """Calculate settling time - time to stay within threshold."""
    abs_errors = np.abs(errors)
    
    # Find first time error goes below threshold
    below_threshold = abs_errors < threshold
    
    if not np.any(below_threshold):
        return None
    
    # Find last time it exceeds threshold
    for i in range(len(below_threshold) - 1, -1, -1):
        if not below_threshold[i]:
            settling_idx = i + 1
            if settling_idx < len(timestamps):
                return (timestamps[settling_idx] - timestamps[0]) / 1000.0
            break
    
    return 0

def calculate_rise_time(errors, timestamps):
    """Calculate rise time from 10% to 90% of final value."""
    if len(errors) < 10:
        return None
    
    final_value = np.mean(errors[-10:])
    
    # Find 10% and 90% points
    ten_percent = 0.1 * final_value
    ninety_percent = 0.9 * final_value
    
    t10_idx = np.argmax(errors >= ten_percent) if final_value > 0 else np.argmax(errors <= ten_percent)
    t90_idx = np.argmax(errors >= ninety_percent) if final_value > 0 else np.argmax(errors <= ninety_percent)
    
    if t10_idx < t90_idx and t90_idx > 0:
        return (timestamps[t90_idx] - timestamps[t10_idx]) / 1000.0
    
    return None

def calculate_overshoot(errors):
    """Calculate maximum overshoot percentage."""
    if len(errors) < 5:
        return 0
    
    steady_state = np.mean(errors[-10:])
    max_error = np.max(np.abs(errors))
    
    if abs(steady_state) < 0.001:
        return max_error
    
    overshoot = (max_error - abs(steady_state)) / abs(steady_state) * 100
    return max(0, overshoot)

def ziegler_nichols_tuning(Ku, Tu):
    """Ziegler-Nichols tuning method."""
    # Classic PID
    Kp = 0.6 * Ku
    Ki = 2 * Kp / Tu
    Kd = Kp * Tu / 8
    
    # Pessen Integral Rule (less overshoot)
    Kp_pessen = 0.7 * Ku
    Ki_pessen = 2.5 * Kp_pessen / Tu
    Kd_pessen = 0.15 * Kp_pessen * Tu
    
    # Some overshoot
    Kp_some = 0.33 * Ku
    Ki_some = 2 * Kp_some / Tu
    Kd_some = Kp_some * Tu / 3
    
    # No overshoot
    Kp_no = 0.2 * Ku
    Ki_no = 2 * Kp_no / Tu
    Kd_no = Kp_no * Tu / 3
    
    return {
        'classic': (Kp, Ki, Kd),
        'pessen': (Kp_pessen, Ki_pessen, Kd_pessen),
        'some_overshoot': (Kp_some, Ki_some, Kd_some),
        'no_overshoot': (Kp_no, Ki_no, Kd_no)
    }

def cohen_coon_tuning(K, tau, theta):
    """Cohen-Coon tuning method for processes with delay."""
    Kp = (1/K) * (tau/theta) * (4/3 + theta/(4*tau))
    Ki = Kp / (theta * (32 + 6*theta/tau) / (13 + 8*theta/tau))
    Kd = Kp * theta * 4 / (11 + 2*theta/tau)
    
    return Kp, Ki, Kd

def analyze_control_quality(errors, corrections, timestamps):
    """Comprehensive control quality analysis."""
    metrics = {}
    
    # Performance indices
    metrics['IAE'] = calculate_iae(errors)
    metrics['ISE'] = calculate_ise(errors)
    
    time_norm = np.array([(t - timestamps[0])/1000.0 for t in timestamps])
    metrics['ITAE'] = calculate_itae(errors, time_norm)
    
    # Statistical measures
    metrics['mean_error'] = np.mean(errors)
    metrics['abs_mean_error'] = np.mean(np.abs(errors))
    metrics['std_error'] = np.std(errors)
    metrics['max_error'] = np.max(np.abs(errors))
    metrics['rms_error'] = np.sqrt(np.mean(np.square(errors)))
    
    # Control effort
    metrics['mean_correction'] = np.mean(np.abs(corrections))
    metrics['max_correction'] = np.max(np.abs(corrections))
    metrics['correction_variability'] = np.std(corrections)
    
    # Dynamic characteristics
    metrics['overshoot'] = calculate_overshoot(errors)
    metrics['settling_time'] = calculate_settling_time(errors, timestamps)
    metrics['rise_time'] = calculate_rise_time(errors, timestamps)
    
    # Oscillation analysis
    osc_freq, osc_power, is_osc = detect_oscillations(errors, timestamps)
    metrics['oscillation_frequency'] = osc_freq
    metrics['oscillation_power'] = osc_power
    metrics['is_oscillating'] = is_osc
    
    return metrics

def intelligent_pid_suggestions(metrics, current_kp, current_ki, current_kd):
    """Intelligent PID tuning suggestions based on multiple metrics."""
    suggestions = []
    priority_scores = []
    
    # Rule-based expert system
    
    # 1. Check for critical oscillations
    if metrics['is_oscillating'] and metrics['oscillation_power'] > 0.1:
        suggestions.append({
            'issue': 'CRITICAL: System is oscillating',
            'diagnosis': f"Dominant oscillation at {metrics['oscillation_frequency']:.2f} Hz with high amplitude",
            'action': 'Reduce proportional gain',
            'recommendation': f"Kp: {current_kp * 0.7:.1f} (reduce by 30%)",
            'priority': 'HIGH'
        })
        priority_scores.append(10)
    
    # 2. Check for underdamping (high overshoot)
    if metrics['overshoot'] > 25:
        suggestions.append({
            'issue': f'High overshoot ({metrics["overshoot"]:.1f}%)',
            'diagnosis': 'System is underdamped - too aggressive response',
            'action': 'Increase derivative gain for damping',
            'recommendation': f"Kd: {current_kd * 1.5:.1f} (increase by 50%)",
            'priority': 'HIGH'
        })
        priority_scores.append(8)
    
    # 3. Check for steady-state error
    if abs(metrics['mean_error']) > 0.03 and metrics['std_error'] < 0.05:
        suggestions.append({
            'issue': f'Persistent steady-state error ({metrics["mean_error"]:.4f})',
            'diagnosis': 'System has offset that proportional control cannot eliminate',
            'action': 'Add integral control',
            'recommendation': f"Ki: {max(0.5, current_ki * 2):.1f} (start with 0.5-1.0 if zero)",
            'priority': 'MEDIUM'
        })
        priority_scores.append(6)
    
    # 4. Check for slow response (high settling time)
    if metrics['settling_time'] is not None and metrics['settling_time'] > 2.0:
        if not metrics['is_oscillating']:  # Only increase Kp if not oscillating
            suggestions.append({
                'issue': f'Slow response (settling time: {metrics["settling_time"]:.2f}s)',
                'diagnosis': 'System responds too slowly to disturbances',
                'action': 'Increase proportional gain',
                'recommendation': f"Kp: {current_kp * 1.2:.1f} (increase by 20%)",
                'priority': 'MEDIUM'
            })
            priority_scores.append(5)
    
    # 5. Check for high error variability
    if metrics['std_error'] > 0.08 and not metrics['is_oscillating']:
        suggestions.append({
            'issue': f'High error variability (std: {metrics["std_error"]:.4f})',
            'diagnosis': 'System is too sensitive to noise/disturbances',
            'action': 'Increase derivative gain for stability',
            'recommendation': f"Kd: {current_kd * 1.3:.1f} (increase by 30%)",
            'priority': 'MEDIUM'
        })
        priority_scores.append(4)
    
    # 6. Check for excessive control effort
    if metrics['correction_variability'] > 50:
        suggestions.append({
            'issue': 'Excessive control effort variation',
            'diagnosis': 'Motors are working too hard - reduces efficiency and wears components',
            'action': 'Smooth control response',
            'recommendation': f"Consider reducing both Kp to {current_kp * 0.9:.1f} and Kd to {current_kd * 0.9:.1f}",
            'priority': 'LOW'
        })
        priority_scores.append(2)
    
    # 7. Check if system is well-tuned
    if (metrics['abs_mean_error'] < 0.03 and 
        metrics['std_error'] < 0.05 and 
        not metrics['is_oscillating'] and
        metrics['overshoot'] < 15):
        suggestions.append({
            'issue': 'System is well-tuned',
            'diagnosis': 'All metrics are within acceptable ranges',
            'action': 'No changes needed',
            'recommendation': f"Current values (Kp={current_kp}, Ki={current_ki}, Kd={current_kd}) are good",
            'priority': 'INFO'
        })
        priority_scores.append(0)
    
    # Sort by priority
    if suggestions and priority_scores:
        sorted_suggestions = [s for _, s in sorted(zip(priority_scores, suggestions), reverse=True)]
        return sorted_suggestions
    
    return suggestions

def estimate_ultimate_gain(errors, corrections):
    """Estimate ultimate gain for Ziegler-Nichols tuning."""
    if len(errors) < 20:
        return None, None
    
    # Find peaks in error signal
    peaks, _ = signal.find_peaks(np.abs(errors), distance=5)
    
    if len(peaks) < 3:
        return None, None
    
    # Estimate oscillation period
    peak_distances = np.diff(peaks)
    if len(peak_distances) > 0:
        Tu = np.mean(peak_distances) * np.mean(np.diff(range(len(errors))))
        
        # Estimate ultimate gain from amplitude ratio
        error_amplitude = np.mean([abs(errors[p]) for p in peaks])
        correction_amplitude = np.mean([abs(corrections[p]) for p in peaks])
        
        if error_amplitude > 0.001:
            Ku = correction_amplitude / error_amplitude
            return Ku, Tu
    
    return None, None

def plot_advanced_analysis(timestamps, errors, corrections, line_positions, motor1, motor2, metrics):
    """Create comprehensive visualization with analysis."""
    t = [(ts - timestamps[0]) / 1000.0 for ts in timestamps]
    
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(4, 2, hspace=0.3, wspace=0.3)
    
    # Error over time
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t, errors, 'r-', linewidth=1.5, label='Error')
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.fill_between(t, errors, 0, alpha=0.2, color='red')
    ax1.set_ylabel('Error')
    ax1.set_title(f'Error Signal (RMS: {metrics["rms_error"]:.4f}, Mean: {metrics["mean_error"]:.4f})', fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # PID output
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(t, corrections, 'b-', linewidth=1.5)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_ylabel('Correction')
    ax2.set_title(f'PID Output (Mean: {metrics["mean_correction"]:.2f})', fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # Motor outputs
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(t, motor1, 'c-', linewidth=1.5, label='Motor 1', alpha=0.7)
    ax3.plot(t, motor2, 'm-', linewidth=1.5, label='Motor 2', alpha=0.7)
    ax3.set_ylabel('Motor Speed')
    ax3.set_title('Motor Outputs', fontweight='bold')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Error distribution histogram
    ax4 = fig.add_subplot(gs[2, 0])
    ax4.hist(errors, bins=30, color='orange', alpha=0.7, edgecolor='black')
    ax4.axvline(x=0, color='r', linestyle='--', linewidth=2, label='Zero error')
    ax4.set_xlabel('Error')
    ax4.set_ylabel('Frequency')
    ax4.set_title(f'Error Distribution (σ={metrics["std_error"]:.4f})', fontweight='bold')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # Frequency spectrum
    ax5 = fig.add_subplot(gs[2, 1])
    if len(errors) > 10:
        detrended = signal.detrend(errors)
        N = len(detrended)
        dt = np.mean(np.diff(t))
        yf = fft(detrended)
        xf = fftfreq(N, dt)[:N//2]
        power = 2.0/N * np.abs(yf[:N//2])
        
        ax5.plot(xf[1:], power[1:], 'g-', linewidth=1.5)
        if metrics['is_oscillating']:
            ax5.axvline(x=metrics['oscillation_frequency'], color='r', 
                       linestyle='--', linewidth=2, 
                       label=f"Dominant: {metrics['oscillation_frequency']:.2f} Hz")
        ax5.set_xlabel('Frequency (Hz)')
        ax5.set_ylabel('Power')
        ax5.set_title('Frequency Spectrum', fontweight='bold')
        ax5.set_xlim(0, min(10, max(xf)))
        ax5.legend()
        ax5.grid(True, alpha=0.3)
    
    # Error vs Correction (phase plot)
    ax6 = fig.add_subplot(gs[3, 0])
    scatter = ax6.scatter(errors, corrections, c=range(len(errors)), 
                         cmap='viridis', alpha=0.5, s=10)
    ax6.set_xlabel('Error')
    ax6.set_ylabel('Correction')
    ax6.set_title('Phase Plot (Error vs Correction)', fontweight='bold')
    ax6.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax6, label='Time progression')
    
    # Line tracking quality
    ax7 = fig.add_subplot(gs[3, 1])
    ax7.plot(t, line_positions, 'g-', linewidth=1.5)
    ax7.axhline(y=0, color='k', linestyle='--', alpha=0.3, label='Center line')
    ax7.fill_between(t, -0.05, 0.05, alpha=0.2, color='green', label='Good tracking zone')
    ax7.set_xlabel('Time (seconds)')
    ax7.set_ylabel('Line Position')
    ax7.set_title('Line Tracking Performance', fontweight='bold')
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    
    plt.suptitle('Advanced PID Control Analysis', fontsize=16, fontweight='bold', y=0.995)
    plt.savefig('pid_advanced_analysis.png', dpi=150, bbox_inches='tight')
    print("\n✓ Advanced plot saved as 'pid_advanced_analysis.png'")
    plt.show()

def main():
    print("="*70)
    print("ADVANCED PID TUNING ANALYZER FOR LINE FOLLOWING ROBOT")
    print("="*70)
    
    # Parse data
    print("\n[1/4] Parsing log.txt...")
    data = parse_log_file('log.txt')
    
    if not data:
        print("❌ No valid data found in log.txt!")
        return
    
    print(f"✓ Loaded {len(data)} samples where robot was running")
    
    # Extract data
    timestamps = [d['timeStamp'] for d in data]
    errors = [d['error'] for d in data]
    corrections = [d['correction'] for d in data]
    line_positions = [d['linePosition'] for d in data]
    motor1 = [d['motor1'] for d in data]
    motor2 = [d['motor2'] for d in data]
    
    current_kp = data[0]['kp']
    current_ki = data[0]['ki']
    current_kd = data[0]['kd']
    
    # Analyze
    print("\n[2/4] Analyzing control performance...")
    metrics = analyze_control_quality(errors, corrections, timestamps)
    
    # Display metrics
    print("\n" + "="*70)
    print("PERFORMANCE METRICS")
    print("="*70)
    print(f"\n📊 Error Metrics:")
    print(f"   Mean Absolute Error: {metrics['abs_mean_error']:.4f}")
    print(f"   RMS Error:           {metrics['rms_error']:.4f}")
    print(f"   Standard Deviation:  {metrics['std_error']:.4f}")
    print(f"   Maximum Error:       {metrics['max_error']:.4f}")
    
    print(f"\n📈 Performance Indices:")
    print(f"   IAE (Integral Absolute Error):     {metrics['IAE']:.2f}")
    print(f"   ISE (Integral Squared Error):      {metrics['ISE']:.2f}")
    print(f"   ITAE (Time-weighted Error):        {metrics['ITAE']:.2f}")
    
    print(f"\n⚙️  Control Effort:")
    print(f"   Mean Correction:     {metrics['mean_correction']:.2f}")
    print(f"   Max Correction:      {metrics['max_correction']:.2f}")
    print(f"   Variability:         {metrics['correction_variability']:.2f}")
    
    print(f"\n🎯 Dynamic Response:")
    if metrics['settling_time']:
        print(f"   Settling Time:       {metrics['settling_time']:.3f}s")
    if metrics['rise_time']:
        print(f"   Rise Time:           {metrics['rise_time']:.3f}s")
    print(f"   Overshoot:           {metrics['overshoot']:.1f}%")
    
    print(f"\n🌊 Oscillation Analysis:")
    if metrics['is_oscillating']:
        print(f"   ⚠️  OSCILLATING at {metrics['oscillation_frequency']:.2f} Hz")
        print(f"   Oscillation Power:   {metrics['oscillation_power']:.4f}")
    else:
        print(f"   ✓ No significant oscillations detected")
    
    # Get intelligent suggestions
    print("\n[3/4] Generating tuning recommendations...")
    suggestions = intelligent_pid_suggestions(metrics, current_kp, current_ki, current_kd)
    
    print("\n" + "="*70)
    print("INTELLIGENT TUNING RECOMMENDATIONS")
    print("="*70)
    print(f"\nCurrent PID: Kp={current_kp}, Ki={current_ki}, Kd={current_kd}\n")
    
    for i, suggestion in enumerate(suggestions, 1):
        priority_symbols = {
            'HIGH': '🔴',
            'MEDIUM': '🟡',
            'LOW': '🟢',
            'INFO': 'ℹ️'
        }
        symbol = priority_symbols.get(suggestion['priority'], '•')
        
        print(f"{symbol} [{suggestion['priority']}] {suggestion['issue']}")
        print(f"   Diagnosis: {suggestion['diagnosis']}")
        print(f"   Action: {suggestion['action']}")
        print(f"   → {suggestion['recommendation']}")
        print()
    
    # Advanced tuning methods
    Ku, Tu = estimate_ultimate_gain(errors, corrections)
    if Ku and Tu:
        print("\n" + "="*70)
        print("ZIEGLER-NICHOLS TUNING SUGGESTIONS")
        print("="*70)
        print(f"Estimated Ultimate Gain (Ku): {Ku:.2f}")
        print(f"Estimated Ultimate Period (Tu): {Tu:.4f}s\n")
        
        zn_methods = ziegler_nichols_tuning(Ku, Tu)
        print("Different tuning approaches:")
        for method, (kp, ki, kd) in zn_methods.items():
            print(f"  {method.replace('_', ' ').title()}: Kp={kp:.1f}, Ki={ki:.2f}, Kd={kd:.2f}")
    
    # Visualization
    print("\n[4/4] Generating visualizations...")
    plot_advanced_analysis(timestamps, errors, corrections, line_positions, 
                          motor1, motor2, metrics)
    
    print("\n" + "="*70)
    print("✓ Analysis complete!")
    print("="*70)

if __name__ == "__main__":
    main()