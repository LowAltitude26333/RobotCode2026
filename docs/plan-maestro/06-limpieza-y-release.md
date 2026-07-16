# 06 — Limpieza y release

> Estado: plan aprobado; ejecución bloqueada hasta aceptar MP-08
> Baseline de referencia: `main` en `f91af18`
> Última actualización: 2026-07-15
> Alcance: snapshot Git, eliminación de legado, build/deploy y rollback
> Responsable sugerido: release lead con revisión del líder de software
> Fuente de verdad: árbol y registros del commit candidato; esta lista no autoriza borrar antes del gate.

## 1. Objetivo

Terminar con una rama `main` enfocada en competencia que sea rápida de entender, difícil de operar mal y recuperable desde Git. El resultado visible en Driver Station será:

- un TeleOp de competencia para Blue y Red;
- un System Check seguro;
- cero autónomos;
- cero tuners/test OpModes registrados en el build normal.

Reducir archivos y dependencias puede mejorar compilación, APK o instalación, pero no se prometerá una mejora de deploy sin medirla. El beneficio garantizado de la limpieza es reducir superficie de error y ruido operativo, no una cifra de segundos.

## 2. Condiciones que bloquean la limpieza

No borrar/mover legado mientras cualquiera sea verdadero:

- Pedro no cumple la calibración acordada;
- Road Runner sigue siendo necesario para recuperar una función activa;
- Limelight/fusión/torreta/shooter no pasan MP-08;
- existe finding crítico/alto abierto;
- no se ejecutaron dos sesiones integradas aceptadas;
- no está terminado el manual;
- no hay permiso explícito para crear/publicar el snapshot y eliminar código;
- el worktree está dirty con cambios no relacionados;
- el tag/branch de rollback no se verificó desde otro checkout/ref.

La limpieza nunca se usa para esconder una implementación fallida.

## 3. Estrategia de preservación

### 3.1 Snapshot obligatorio

En el commit exacto inmediatamente anterior a la eliminación:

1. asegurar worktree limpio;
2. ejecutar build completo y checks;
3. registrar SHA, dependencias resueltas, APK/hash y resultado de smoke test;
4. crear tag anotado con formato `archive/pre-cleanup-YYYYMMDD`;
5. publicar el tag al remoto autorizado;
6. verificar que el tag puede consultarse y construirse desde un checkout/worktree separado;
7. registrar el procedimiento de restauración.

Comandos ilustrativos —reemplazar fecha y no ejecutar sin revisar estado/permisos—:

```powershell
git status --short
git rev-parse HEAD
git tag -a archive/pre-cleanup-YYYYMMDD -m "Complete pre-cleanup competition/commissioning snapshot"
git show --stat archive/pre-cleanup-YYYYMMDD
git push origin archive/pre-cleanup-YYYYMMDD
```

No crear un directorio Java `archive/` dentro de `main`: sus clases podrían seguir compilándose/registrándose y no reducen superficie real. El historial Git es el archivo.

### 3.2 Rama de commissioning

Mantener una rama claramente nombrada para:

- tuners de Pedro/Road Runner que todavía sean útiles;
- OpModes de caracterización complejos;
- replay/log tooling que no se incluya en producción;
- snapshot funcional de Road Runner;
- experimentos que no cumplen gates de release.

La rama no es autoridad de competencia y debe registrar desde qué tag/commit se derivó.

## 4. Inventario antes de borrar

Generar y revisar:

- todos los archivos `TeamCode/src/main/java`;
- anotaciones `@TeleOp`, `@Autonomous`, `@Disabled`;
- registradores dinámicos `@OpModeRegistrar`;
- referencias desde manifest/config/Gradle;
- hardware mappings;
- subsystems/commands sin consumidores;
- assets de VisionPortal/Limelight;
- dependencias usadas y no usadas;
- MeepMeep y módulos de simulación;
- archivos generados/ignorados que no deben entrar al diff.

Probar actividad por call path, no por nombre de archivo.

## 5. Clasificación de componentes

| Categoría | Acción objetivo | Momento |
|---|---|---|
| TeleOp de competencia nuevo | Conservar | Siempre. |
| System Check seguro | Conservar | Siempre. |
| Otros TeleOps habilitados/deshabilitados | Eliminar de `main` tras snapshot | MP-09. |
| Todos los autónomos | Eliminar de `main` tras snapshot | MP-09; no hay autos en scope final. |
| RR tuning registrars | Eliminar/desactivar físicamente del build final | Tras aceptar Pedro. |
| Pedro tuners complejos | Mover por historial/rama, no a package archive | Tras calibración. |
| Road Runner runtime | Eliminar si no hay consumidor activo | Tras aceptar Pedro y snapshot. |
| MeepMeepTesting | Eliminar sólo si quedó sin uso y está autorizado | MP-09, cambio separado si es grande. |
| VisionPortal/AprilTag tracking viejo | Eliminar | Tras Limelight validada. |
| `ShooterHoodSubsystem` y comandos/presets | Eliminar | Tras retiro físico y shooter fijo validado. |
| `Shooter2` legado | Eliminar | Tras confirmar un motor en configuración/código. |
| `KickerSubsystem` | Renombrar conceptualmente a feeder | Antes o durante MP-06; preservar hardware string hasta confirmarlo. |
| Alternate localizers/drives | Eliminar del build final | Tras demostrar que no son utilizados. |
| `FtcRobotController` SDK app | No modificar | Fuera de alcance. |

## 6. Orden de limpieza

Hacer commits pequeños, cada uno compilable:

1. **Registry cleanup:** dejar registrados sólo Competition TeleOp y System Check.
2. **OpMode deletion:** eliminar autos, TeleOps y tuners no usados.
3. **Command/subsystem dead code:** eliminar transitivamente sólo después de búsqueda de referencias.
4. **Vision legacy:** retirar Webcam/VisionPortal/AprilTag custom si Limelight reemplazó todo consumidor.
5. **Hood/shooter legacy:** retirar hood, presets y `Shooter2` después de contrato físico.
6. **Road Runner/MeepMeep:** retirar código y dependencias sólo tras confirmar cero imports/registrars/consumidores.
7. **Dependency consolidation:** limpiar duplicados y fijar la versión de SDK ya efectivamente usada, sin upgrade.
8. **Docs/config:** actualizar inventario, manual, diagrams, licenses y deployment instructions.

No combinar eliminación masiva con cambios funcionales de targeting. Si un commit falla, debe ser obvio qué categoría lo causó.

## 7. Dependencias y versiones

El baseline presenta fuentes superpuestas: `TeamCode/build.gradle` declara SDK 10.3.0 mientras `build.dependencies.gradle` incluye SDK 11.0, además de duplicados de Dashboard. La integración de Limelight depende de la API realmente resuelta.

Política:

- durante MP-01 a MP-08, documentar la resolución efectiva y cambiar sólo lo indispensable;
- en MP-09, consolidar la versión **existente y ya validada** en una sola fuente si puede hacerse sin alterar APIs;
- no actualizar FTC SDK, Gradle, Android Gradle Plugin, Java ni librerías por “estar limpiando”;
- mantener un cambio de versiones en commit/PR separado con build y smoke test completos;
- conservar notices/licencias de dependencias externas.

## 8. Benchmark antes/después

### 8.1 Variables controladas

- misma máquina, JDK/JAVA_HOME y power state;
- mismo daemon/cache policy;
- misma conexión al robot/hub para deploy;
- mismo tipo de build;
- registrar antivirus/Android Studio si afecta;
- al menos tres repeticiones cuando sea posible.

### 8.2 Métricas

| Métrica | Baseline | Candidato | Cambio | Interpretación |
|---|---:|---:|---:|---|
| `assembleDebug` frío | Medir | Medir | Calcular | Incluye configuración/compilación/packaging. |
| `assembleDebug` caliente sin cambios | Medir | Medir | Calcular | Mide overhead incremental. |
| Build tras editar un Java pequeño | Medir | Medir | Calcular | Aproxima ciclo de desarrollo. |
| APK bytes | Medir | Medir | Calcular | No equivale directamente a install time. |
| Install/deploy al RC | Medir | Medir | Calcular | Objetivo que preocupa al usuario. |
| Tiempo hasta OpMode visible | Medir | Medir | Calcular | Incluye restart/scan. |
| Conteo OpModes DS | Medir | 2 | Calcular | Mejora operativa directa. |
| Archivos/clases/deps | Medir | Medir | Calcular | Explica parte del cambio. |

La medición de investigación (`~192.3 s`, APK `81,278,696` bytes) es sólo referencia inicial; repetir bajo protocolo.

## 9. Verificaciones por commit de limpieza

1. `rg` de imports/referencias antes de borrar.
2. Diff completo y `git diff --check`.
3. Enumerar OpModes anotados/dinámicos.
4. Confirmar hardware mappings/owners.
5. Build `assembleDebug`.
6. Inspeccionar APK/registro si corresponde.
7. Instalar candidato.
8. Ver Driver Station: exactamente dos modos esperados, sin colisiones.
9. Smoke test deshabilitado y luego seguro a baja potencia.
10. Registrar métricas/findings.

Si una dependencia “parece” no usada, demostrarlo con imports, Gradle dependency graph y build; no removerla sólo por nombre.

## 10. Release candidate

### 10.1 Identidad

El candidato debe tener:

- versión/nombre visible;
- SHA limpio;
- configuración RC identificada;
- versión de Limelight/pipeline/fieldmap;
- constants/calibration version;
- manual correspondiente;
- changelog y findings conocidos;
- APK/hash guardado según práctica del equipo.

### 10.2 Gate funcional

- build exitoso desde checkout limpio;
- un TeleOp + un System Check visibles;
- cero autos/tuners;
- normal, odometry-only y degraded probados;
- alliance/preset/arm init funcional;
- E-stop/stop paths probados;
- criterios de MP-08 cumplidos;
- dos sesiones integradas aceptadas;
- sin critical/high abierto.

### 10.3 Gate de propiedad del equipo

Al menos dos personas, incluida una estudiante, pueden explicar:

- cómo llega pose a la torreta;
- cuándo Limelight se rechaza;
- qué significa cada modo;
- cómo se calcula/aplica trim de RPM;
- por qué el feeder está bloqueado;
- cómo parar y cómo restaurar el tag.

## 11. Runbook de rollback

### Caso A — Fallo descubierto antes del evento

1. parar el uso del candidato;
2. registrar finding con logs;
3. crear rama desde último release/tag aceptado;
4. aplicar sólo fix pequeño y repetir gates afectados;
5. no mezclar la vuelta con nueva limpieza.

### Caso B — Necesidad de recuperar el pre-cleanup

```powershell
git fetch --tags
git switch -c recovery/pre-cleanup archive/pre-cleanup-YYYYMMDD
$env:JAVA_HOME='C:\Program Files\Android\Android Studio\jbr'
.\gradlew.bat assembleDebug
```

Después verificar configuración y hacer smoke test. Nunca desplegar sólo porque el tag compila; puede requerir la configuración física de aquella fecha.

### Caso C — Pedro falla y se evalúa Road Runner

Road Runner vive en el snapshot, no oculto en producción. Crear una rama desde el tag, verificar hardware/config y ejecutar las pruebas de localización y seguridad correspondientes. No alternar stacks en medio de un match mediante un flag no validado.

## 12. Lo que no se hará

- `git reset --hard` o checkout destructivo sobre trabajo del usuario;
- borrar sin snapshot y permiso;
- guardar legacy compilable en `/archive` dentro de source;
- modificar `FtcRobotController` como limpieza incidental;
- actualizar toolchain/SDK de manera lateral;
- conservar dos composition roots “por si acaso”;
- declarar mejora de deploy sin medir instalación real;
- eliminar documentación, licencias, configuración o logs necesarios para reconstruir el release.

## 13. Checklist final de limpieza

- [ ] MP-08 aceptado y permiso explícito.
- [ ] Worktree limpio y SHA registrado.
- [ ] Tag anotado creado, publicado y verificado.
- [ ] Rama de commissioning definida.
- [ ] Road Runner recuperable desde snapshot.
- [ ] Un TeleOp de competencia.
- [ ] Un System Check seguro.
- [ ] Cero autos/tuners/registrars extra.
- [ ] Cero owner duplicado.
- [ ] VisionPortal/hood/Shooter2/legacy retirado donde corresponde.
- [ ] Dependencias no usadas comprobadas antes de remover.
- [ ] SDK efectivo consolidado sin upgrade incidental.
- [ ] Builds y deploys antes/después medidos.
- [ ] Driver Station verificado.
- [ ] Manual/changelog/license actualizados.
- [ ] Rollback reconstruido desde otro checkout.

## 14. Definición de terminado

La limpieza termina cuando el release pequeño es funcional, medido y recuperable. “El repo tiene menos archivos” no es suficiente. Si el snapshot no puede reconstruirse o el System Check quedó sin seguridad, la limpieza falló aunque el APK sea menor.
