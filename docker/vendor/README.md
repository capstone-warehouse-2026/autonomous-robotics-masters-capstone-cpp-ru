# ONNX Runtime 1.31.0 для Linux x86-64

Архив содержит CPU-версию ONNX Runtime из каталога `/opt/onnxruntime`
локального образа `capstone-dev:local`. При исходной сборке Dockerfile
проверял SHA256 архива релиза Microsoft
`cc5c72baf5ae5c8238a6841f0897227be2d02826b9cf98eaf02fdefaeeb03a57`.
Идентификатор исходников из файла `GIT_COMMIT_ID` —
`402979ac1bef89dcd55add41eaa7277c407599eb`.

В архиве сохранены `LICENSE`, `ThirdPartyNotices.txt` и метаданные поставки.
При перепаковке исключены две ссылки, которые Dockerfile создаёт сам
(`include/onnxruntime` и `lib64`). SHA256 перепакованного файла —
`2bd39b28a2708bb7e37f5cc83ecd7089dcb0a90f9a030c9fa0ece0fe8be26ea1`.
Dockerfile проверяет его перед распаковкой.
