# MT XSens Xbus MTData2 Data Stream Decoder

Date: 2022-12-29

- [ ] [資料](https://base.xsens.com/s/article/How-to-use-Device-Data-View-to-learn-MT-Low-Level-Communications?language=en_US)
- [ ] 在用 double 接收資料時，重複開開關關 USE Serial 好像會有問題，系統會直接當機，原因是? 要設 timeout 時間到就 print 現在情況，然後讀 buffer 要不要加保護? 看起來好像沒辦法，因為系統是直接死去，例外處理呢? 其實不是死去，應該是 buffer 不夠

## Xbus-MTData2 Data Structure
