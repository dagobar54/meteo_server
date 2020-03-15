# meteo_server
meteo for home. Common receiver. stm32f103c8t6
Это часть личного проекта по созданию распределенной метеостанции для дома.
Метеостанция организована в сеть на основе дешевого радиомодуля nRF2401+.
Этот модуль позволяет легко объеденить в одну сеть 5 клиентских модулей и один серверный.
Для обмена используется технология FastBoost, при которой передатчик в одном сеансе обмена с приемником передает пакет данныз и получает пакент в ответю
Предполагается 2 одиковых датчика на основе BMT280  с батарейным питание, работающие на arduino pro mini.
Один из датчиков размещается под подоконником, а второй в жилом помещении.
Предполагается клиент, имеющий доступ в интернет для получения точного времени и, возможно, прогноза погоды.
Два индикаторных клиента: один с батарейным питанием на e-ink экране и еще один на LCD ярком экране с сетевым питанием.
Приемный сервер также имеет сетевое питание.
