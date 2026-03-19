# Arquitectura y Justificación del Filtro Complementario (Día 6)

## 1. El Problema de la Fusión de Sensores
El MPU6050 proporciona datos de aceleración y velocidad angular, pero ninguno es confiable por sí solo en un entorno de vuelo (alta vibración):
* **Acelerómetro:** Es preciso a largo plazo (referencia gravitacional absoluta), pero extremadamente susceptible al ruido de alta frecuencia (vibración de los motores coreless).
* **Giroscopio:** Es preciso a corto plazo y no le afectan las vibraciones lineales, pero sufre de *Drift* (deriva de baja frecuencia) al integrar pequeños errores estáticos a lo largo del tiempo.

## 2. Ecuación del Filtro Complementario
El filtro combina las fortalezas de ambos sensores en el dominio del tiempo discreto. Se ejecuta a una frecuencia de 100 Hz ($dt = 0.01s$).

$$\theta_{CF} = \alpha \times (\theta_{CF-prev} + \omega_{gyro} \times dt) + (1 - \alpha) \times \theta_{accel}$$

* $\theta_{accel} = \arctan\left(\frac{A_y}{A_z}\right) \times \frac{180}{\pi}$
* $\omega_{gyro}$ es la velocidad angular en º/s.

## 3. Justificación del Coeficiente Alfa ($\alpha = 0.98$)
El valor de $\alpha$ no es un número mágico; define un par de filtros físicos: un filtro Pasa-Altos para el giroscopio y un filtro Pasa-Bajos para el acelerómetro.

La relación entre $\alpha$, el intervalo de tiempo ($dt$) y la constante de tiempo del filtro ($\tau$) está dada por:
$$\alpha = \frac{\tau}{\tau + dt}$$

Despejando $\tau$ para nuestro sistema donde $\alpha = 0.98$ y $dt = 0.01$ s:
$$0.98 = \frac{\tau}{\tau + 0.01} \implies \tau = 0.49 \text{ segundos}$$

Esto significa que el filtro confía en el giroscopio para movimientos que duran menos de medio segundo, y confía en el acelerómetro para inclinaciones sostenidas.

**La Frecuencia de Corte ($f_c$):**
$$f_c = \frac{1}{2 \pi \tau} = \frac{1}{2 \pi (0.49)} \approx 0.32 \text{ Hz}$$

Cualquier vibración del motor superior a 0.32 Hz reportada por el acelerómetro es rechazada matemáticamente por el filtro, garantizando que el controlador PID no reaccione al ruido mecánico.

## 4. Calibración y Offsets
Antes de aplicar la fusión, el hardware requiere una calibración para mitigar el error constante de fábrica (Bias). 
Durante la programacion del código se tomo una muestra de 35 segundo a 100Hz (3500 muestras) y se al archivo calibracion_mpu6050.m para obtener los offset con MATLAB, garantizando que $\omega_{gyro}$ sea exactamente $0^\circ/s$ antes de la integración.