# 3-DoF RRR Planar Robot Analizi

Bu proje, 3 Serbestlik Derecesine (3-DoF) sahip bir RRR Planar (Düzlemsel) Robot Kolunun kinetik analizini, çalışma alanı (workspace) görselleştirmesini ve mekanik simülasyonunu içermektedir.

## Özellikler

- **Robot Seçimi**: RRR Planar Kol. (Tabandan 2D düzlemde hareket eden 3 adet döner mafsal içeren sistem).
- **İleri Kinematik (FK)**: Standart Denavit-Hartenberg (DH) yöntemine göre hesaplanmış Dönüşüm Matrisleri.
- **Ters Kinematik (IK)**: Analitik kosinüs teoremi ile iteratif olmayan matematiksel çözüm.
- **Çalışma Alanı Görselleştirme**: Robotun mafsal limitleri dahilinde ulaşabileceği tüm XYZ noktalarının Matplotlib ile projeksiyonu (Point Cloud).
- **Simülasyon Animasyonu**: Belirlenen hedef noktaya (0.8, 0.2, 0.1) ulaşmak için robot kolunun hareketinin mekanik GIF animasyonu. 

*Not: Hedef Z=0.1 için, taban d1=0.1 offseti ile düzlemsel hareketin 0.1 yüksekliğinde gerçekleşmesi sağlanmıştır.*

## Kullanılan Teknolojiler

- Python 3
- NumPy `pip install numpy`
- Matplotlib `pip install matplotlib`
- Pillow `pip install pillow` (GIF kaydetmek için)

## Çalıştırma Talimatları

Proje dosyalarını çalıştırıp analizleri görmek için, terminal (veya PowerShell/CMD) açıp çalıştırın:
```bash
python rrr.py
```
Bu işlem sonunda klasörünüzde şu dosyalar oluşacaktır:
1. `workspace.png` : Çalışma alanı grafiği.
2. `simulation.gif` : Hedef noktaya hareketin animasyonu.
3. `robot_final_pose.png` : Hedefe ulaşmış robotun statik hali.

## Kullanılan Kaynaklar
- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson Prentice Hall.
- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). *Robot Modeling and Control* (2nd ed.). Wiley.
