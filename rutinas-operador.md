# Rutinas de operador

Vías aparte del ciclo automático. Los pasos 1 a 8 del ciclo no cambian y no se duplican aquí. La pantalla manda estas rutinas; Motion mueve el eje y los alimentadores de corte; el PLC maneja aire, pinzas y cortador.

Hasta terminar la revisión de arranque, **Start** queda bloqueado. En banco se puede saltar con `HMI_CHECKLIST_SKIP=1`.

<!-- ANDON_SKIP_OPCIONAL — TODO_BORRAR: párrafo siguiente (puente sin cable Andon) -->

Si no hay segundo cable USB para el Andon: en **Parámetros → Conexión** elige **No usar Andon** (se guarda en `data/serial.json`), o arranca el HMI con `HMI_ANDON_SKIP=1`. En ambos casos no exige COM del Andon para el ciclo (Motion y PLC siguen obligatorios).

<!-- fin ANDON_SKIP_OPCIONAL -->

## Purga

Saca el tubo de la zona de corte. No cuenta como ciclo de producción.

1. Apaga el aire y lleva el eje a parque, 250 mm.
2. Queda en espera. El aviso y los botones salen en el banner superior:
   - **Alimentar.** Cierra holder y encoder juntos y corre el mismo feed stage del ciclo: ambos lados, láser y validación de encoder. Se puede repetir.
   - **Limpiar.** Cierra holder y encoder juntos y alimenta 3 s sin mirar el láser. Se puede repetir.
   - **Continuar.** Cierra pinzas, pulsa los cortadores el mismo tiempo del ciclo, espera 500 ms, va a la posición de descarga (si está definida), abre pinzas, regresa a 0 con soplo y cierra la purga.
   - **Terminar.** Apaga el aire y cierra la purga sin cortar. El eje se queda donde está.
3. Si no hay longitud de modelo, Continuar no corta: suelta y regresa el eje a 0.

Mientras corre, no se puede iniciar, reanudar, pausar ni cargar material. **Stop** la cancela y apaga el aire. En el tramo de corte el banner muestra **NO METAS LAS MANOS** y esconde los botones. **Posición inicial** queda bloqueada en ese tramo. Si había un ciclo en pausa, al terminar la pieza vuelve al paso 1 y el ciclo sigue en pausa.

## Posición inicial

Lleva el eje a 0, pone en cero los dos encoders y apaga el aire. Si hay una falla del PLC, intenta el reset; si la falla sigue, no se mueve. Si había un ciclo en pausa, la pieza vuelve al paso 1. Durante la espera de la purga también se puede usar.

## Revisión de arranque

Seis pasos, en orden. Pide un empleado de cinco dígitos. Cerrar la ventana no borra el paso; al reiniciar el programa la sesión a medias se descarta. Al completar se guarda empleado, fecha y resultado.

| Paso | Acción |
|------|--------|
| 1. Contador | Si hay 10 000 ciclos o más desde el último reset de mantenimiento, no deja seguir. |
| 2. Aire | Acciona holder y encoder, lee fallas del PLC y apaga las salidas. |
| 3. Holgura | El operador confirma a ojo. La máquina no lee un sensor. |
| 4. Eje | Referencia mecánica, recorrido hasta la posición de descarga y regreso a 0. El operador confirma que regresó. |
| 5. Purga | Inicializa los alimentadores y corre la purga de arriba. |
| 6. Piezas | Corta dos piezas del modelo cargado. Cada lado debe medir la longitud del modelo más 55 mm, dentro de la tolerancia del ciclo. |

## Carga de material

Alterna el modo de carga. Mientras está activo, **Start** y **Reanudar** quedan bloqueados. Si había un ciclo en pausa, la pieza vuelve al paso 1. No se puede activar durante una purga.

## Iniciar

Inicializa los dos servos de alimentación. Es el mismo arranque que usa el paso 5 de la revisión.

## Alimentar izquierda / derecha

Mueve solo el alimentador de ese lado, con la misma validación de láser y encoder que el paso 3 del ciclo. No mueve el eje y no corta. Hace falta un modelo con longitud. El botón queda activo mientras ese lado avanza.

## Omitir holgura

Cada lado es independiente y pide el empleado de cinco dígitos. La omisión queda en la pantalla hasta reiniciar el programa. Pulsar otra vez restaura la exigencia. En este equipo no hay esclavo de holgura: el estado se usa en la HMI y en el registro.

## Mantenimiento

Muestra los ciclos terminados desde el último reset. Resetear pide empleado de cinco dígitos y deja el contador en cero. Cada ciclo completado, incluidas las dos piezas de prueba, suma uno.
