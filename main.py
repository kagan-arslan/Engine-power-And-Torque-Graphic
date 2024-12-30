import numpy as np
import matplotlib.pyplot as plt

class EngineModel:
    """
    Motor modelini tanımlayan sınıf.
    Tork ve güç eğrisini motor devrine (RPM) göre hesaplar.
    """
    def __init__(self, max_torque, max_power, max_rpm, efficiency=0.85):
        self.max_torque = max_torque  # Nm
        self.max_power = max_power  # kW
        self.max_rpm = max_rpm  # RPM
        self.efficiency = efficiency  # Motor verimliliği

    def torque(self, rpm):
        """
        Motorun torkunu RPM'ye göre hesaplar.
        Zirve torktan sonra düşüş yaşanır.
        """
        if rpm <= 0 or rpm > self.max_rpm:
            return 0
        peak_rpm = self.max_rpm * 0.5
        if rpm <= peak_rpm:
            return self.max_torque * (rpm / peak_rpm)  # Zirveye kadar artış
        else:
            return self.max_torque * (1 - (rpm - peak_rpm) / (self.max_rpm - peak_rpm))  # Zirve sonrası düşüş

    def power(self, rpm):
        """
        Motor gücünü RPM'ye göre hesaplar (kW).
        """
        torque = self.torque(rpm)
        return (torque * rpm) / 9548.8

    def efficiency_factor(self, rpm):
        """
        Motor verimlilik faktörünü hesaplar.
        Minimum verimlilik sınırı: 0.6
        """
        efficiency = self.efficiency - 0.001 * (rpm / self.max_rpm)
        return max(efficiency, 0.6)

class TransmissionSystem:
    """
    Şanzıman sistemini tanımlayan sınıf.
    """
    def __init__(self, gears, efficiency=0.95):
        self.gears = gears
        self.efficiency = efficiency
        self.current_gear = 0  # Başlangıçta 1. viteste

    def change_gear(self, rpm, max_rpm):
        """
        Uygun vitese geçişi RPM'ye bağlı olarak hesaplar.
        """
        if rpm > max_rpm * 0.85 and self.current_gear < len(self.gears) - 1:
            self.current_gear += 1  # Yüksek vitese geç
        elif rpm < max_rpm * 0.3 and self.current_gear > 0:
            self.current_gear -= 1  # Düşük vitese geç
        return self.gears[self.current_gear]

    def get_gear_ratio(self):
        """
        Mevcut vites oranını döndürür.
        """
        return self.gears[self.current_gear]

class VehicleDynamics:
    """
    Araç dinamiklerini tanımlayan sınıf.
    """
    def __init__(self, weight, drag_coefficient, frontal_area, engine, transmission, tire_grip=0.9):
        self.weight = weight  # kg
        self.drag_coefficient = drag_coefficient
        self.frontal_area = frontal_area  # m^2
        self.air_density = 1.225  # kg/m^3 (hava yoğunluğu)
        self.engine = engine
        self.transmission = transmission
        self.tire_grip = tire_grip  # Lastik tutuş katsayısı
        self.velocity = 0.1  # Başlangıç hızı (m/s)
        self.position = 0  # Başlangıç pozisyonu (m)
        self.time_step = 0.1  # Simülasyon adım zamanı (s)
        self.acceleration = 0  # Başlangıç ivmesi (m/s²)
        self.velocity_limit = 200 / 3.6  # Maksimum hız (m/s)
        self.is_braking = False  # Frenleme durumu
        self.slope_angle = 0  # Yol eğimi (derece)

    def update(self, rpm, slope=0):
        """
        Her adımda hız, pozisyon ve ivmeyi günceller.
        """
        self.slope_angle = slope  # Yol eğimini güncelle
        
        # Motorun sağladığı güç ve şanzıman oranı
        gear_ratio = self.transmission.change_gear(rpm, self.engine.max_rpm)
        power = self.engine.power(rpm) * self.engine.efficiency_factor(rpm)

        # Kuvvet hesapları
        force = (power * 1000 * gear_ratio * self.transmission.efficiency) / self.velocity if self.velocity > 0 else 0
        drag_force = 0.5 * self.drag_coefficient * self.air_density * self.frontal_area * self.velocity**2
        gravity_force = self.weight * 9.81 * np.sin(np.radians(self.slope_angle))
        net_force = force - drag_force - gravity_force
        
        # Lastik tutuş kaybını kontrol et
        if np.abs(net_force) > self.tire_grip * self.weight * 9.81:
            net_force *= self.tire_grip

        self.acceleration = net_force / self.weight

        # Frenleme durumunda negatif ivme uygula
        if self.is_braking:
            self.acceleration -= 5

        # Hız ve pozisyon güncelleme
        self.velocity = max(0, min(self.velocity + self.acceleration * self.time_step, self.velocity_limit))
        self.position += self.velocity * self.time_step

        return self.velocity, self.position, self.acceleration

# Parametreler
motor = EngineModel(max_torque=500, max_power=300, max_rpm=6000)
transmission = TransmissionSystem(gears=[4, 2.5, 1.5, 1])
vehicle = VehicleDynamics(weight=1500, drag_coefficient=0.32, frontal_area=2.2, engine=motor, transmission=transmission)

# Simülasyon
time_duration = 200
rpm_range = np.linspace(1000, 6000, time_duration)
velocities = []
positions = []
accelerations = []
slopes = np.zeros(time_duration)

# Yol eğimi örneği
for i in range(time_duration):
    if 50 <= i <= 100:  # 50-100 adımları arasında %5 eğim
        slopes[i] = 5
    elif 150 <= i <= 180:  # 150-180 adımları arasında %5 iniş
        slopes[i] = -5
    velocity, position, acceleration = vehicle.update(rpm_range[i], slope=slopes[i])
    velocities.append(velocity)
    positions.append(position)
    accelerations.append(acceleration)

# Grafikler
plt.figure(figsize=(12, 8))

# Hız Grafiği
plt.subplot(3, 1, 1)
plt.plot(velocities, label="Hız (m/s)", color="b")
plt.title("Araç Hızı Zamanla")
plt.xlabel("Zaman Adımı (s)")
plt.ylabel("Hız (m/s)")
plt.grid(True)
plt.legend()

# Pozisyon Grafiği
plt.subplot(3, 1, 2)
plt.plot(positions, label="Pozisyon (m)", color="r")
plt.title("Araç Pozisyonu Zamanla")
plt.xlabel("Zaman Adımı (s)")
plt.ylabel("Pozisyon (m)")
plt.grid(True)
plt.legend()

# İvme Grafiği
plt.subplot(3, 1, 3)
plt.plot(accelerations, label="İvme (m/s²)", color="g")
plt.title("Araç İvmesi Zamanla")
plt.xlabel("Zaman Adımı (s)")
plt.ylabel("İvme (m/s²)")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()