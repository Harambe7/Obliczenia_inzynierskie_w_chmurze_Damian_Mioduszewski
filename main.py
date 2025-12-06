import math
import matplotlib.pyplot as plt
import random
import time

# Założyłem dane dla BMW M3 G80 3.0l Competition xDrive  https://www.carfolio.com/bmw-m3-804188

predkosc_poczatkowa = 80
predkosc_zadana = 130
czas_symulacji = 60
krok_symulacji = 0.01 # Zmienilem na 0.01 dla plynnosci (0.1 przy zmniejszaniu pr. wprowadzal oscylacje)
liczba_iteracji_petli = 10000
kat_nachylenia_gorki = 10.0
limit_gazu_komfort = 0.65 # maks 1.0
limit_hamulca_komfort = -0.5 # maks -1.0

# Klasa regulatora PID
class PID:
    def __init__(self, p, i, d):
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.last_err = 0 # uchyb z poprzedniego kroku
        self.calka = 0

    def licz(self, cel, aktualna, dt):
        e = cel - aktualna
        self.calka += (e * dt)
        
        # zabezpieczenie przed dzieleniem przez 0
        if dt > 0:
            der = (e - self.last_err) / dt  # pochodna z uchybu
        else:
            der = 0.0
        
        # wzór na PID
        u = (self.Kp * e) + (self.Ki * self.calka) + (self.Kd * der)
        self.last_err = e
        
        if u > limit_gazu_komfort: return limit_gazu_komfort
        
        # Hamulec, ograniczony do 50% dla komfortu pasażerów
        if u < limit_hamulca_komfort: return limit_hamulca_komfort
        
        return u

# Założyłem dane dla BMW M3 G80 3.0l Competition xDrive  https://www.carfolio.com/bmw-m3-804188
class Auto:
    def __init__(self, v_start_kmh=0.0): 
        self.m = 1855 # masa [kg]
        
        # Zabezpieczenie juz przy tworzeniu auta
        if v_start_kmh > 250.0:
            v_start_kmh = 250.0
            
        self.v = v_start_kmh / 3.6 # prędkość aktualna [m/s]
        self.Cd = 0.33 # wsp opóru aerodynamicznego
        self.A = 2.34 # powierzchnia czołowa [m^2]
        self.rho = 1.225 # gęstość powietrza [kg/m^3]
        
        # Dane silnika i napedu
        self.max_power = 353000 # [W] moc = 353kW
        self.mu = 0.9 # wspolczynnik tarcia (wysoki bo opony sportowe + naped xDrive)

    def jazda(self, gaz, dt, kat_stopnie=0):

        # 1) ograniczenie przez moc: F = P / v
        v_calc = max(abs(self.v), 0.1) 
        F_power = self.max_power / v_calc

        # 2) ograniczenie przez przyczepność: F = mu * m * g
        F_traction = self.mu * self.m * 9.81

        F_available = min(F_power, F_traction) 

        # F_silnik zalezy od tego ile PID dal gazu (-50% do 100%)
        F_silnik = gaz * F_available 

        # opór aerodynamiczny
        F_opor = 0.5 * self.rho * self.Cd * self.A * (self.v ** 2)
        
        # siła grawitacji wzdłuż nachylenia
        F_graw = self.m * 9.81 * math.sin(math.radians(kat_stopnie))
        
        # siła wypadkowa
        F_wyp = F_silnik - F_opor - F_graw
        a = F_wyp / self.m
        
        # aktualizacja prędkości o vn = vn-1 + at do kolejnych iteracji
        self.v += a * dt
        
        # zabezpieczenie
        if self.v < 0: self.v = 0 
        
        # Ograniczenie prędkości maksymalnej
        v_max_ms = 250.0 / 3.6
        if self.v > v_max_ms:
            self.v = v_max_ms

def jeden_przejazd(k_p, k_i, k_d, v_zadana, v_start, czas, dt, kat_gorki):
    cel_ms = v_zadana / 3.6 #na m/s

    moje_auto = Auto(v_start) 
    regulator = PID(k_p, k_i, k_d)
    
    kroki = int(czas / dt)
    
    tab_t = []
    tab_v = []
    tab_cel = []
    
    suma_uchybow = 0 

    for i in range(kroki):
        t = i * dt
        
        nachylenie = 0
        if czas_symulacji/2 < t < czas_symulacji/1.8: #symulacja nachylenia
            nachylenie = kat_gorki
            
        sterowanie = regulator.licz(cel_ms, moje_auto.v, dt)
        moje_auto.jazda(sterowanie, dt, nachylenie)
        
        tab_t.append(t)
        tab_v.append(moje_auto.v * 3.6)
        tab_cel.append(v_zadana)

        # licze blad zeby znalezc najlepsze ustawienia
        roznica = v_zadana - (moje_auto.v * 3.6)
        suma_uchybow += (roznica * roznica)

    return suma_uchybow, tab_t, tab_v, tab_cel

def main():
    start_programu = time.time() 

    v_startowa = predkosc_poczatkowa
    v_zadana = predkosc_zadana
    czas = czas_symulacji
    dt = krok_symulacji
    iteracje = liczba_iteracji_petli
    alfa = kat_nachylenia_gorki
    
    V_MAX_LIMIT = 250.0 # Maksymalna predkosc BMW

    print("Symulacja Tempompatu z wykorzytaniem regulatora PID")
    print("Auto: BMW M3 Competition xDrive (dane z carfolio)\n")
    print(f"Wersja pod chmure (Docker) - {iteracje} iteracji")

    # sprawdzenie poprawnosci podanych danych
    if v_startowa > V_MAX_LIMIT:
        print(f"!!! UWAGA: Predkosc poczatkowa {v_startowa} km/h przekracza blokade!")
        print(f"--> Ograniczam V startowa do {V_MAX_LIMIT} km/h")
        v_startowa = V_MAX_LIMIT

    if v_zadana > V_MAX_LIMIT:
        print(f"!!! UWAGA: Predkosc zadana {v_zadana} km/h przekracza blokade!")
        print(f"--> Ograniczam cel do {V_MAX_LIMIT} km/h")
        v_zadana = V_MAX_LIMIT

    print(f"Scenariusz: Start {v_startowa} km/h -> Cel {v_zadana} km/h")
    print(f"Komfort: Gaz max {limit_gazu_komfort*100}%, Hamulec max {abs(limit_hamulca_komfort)*100}%")

    najlepszy_blad = float('inf') 
    najlepsze_nastawy = (0,0,0)

    # Petla zeby obciazyc maszyne troche
    for k in range(iteracje):
        if k % 2000 == 0:
             print(f"Licze iteracje: {k}")

        los_p = random.uniform(0.1, 8.0)
        los_i = random.uniform(0.0, 2.0)
        los_d = random.uniform(0.0, 5.0)

        blad, _, _, _ = jeden_przejazd(los_p, los_i, los_d, v_zadana, v_startowa, czas, dt, alfa)

        if blad < najlepszy_blad:
            najlepszy_blad = blad
            najlepsze_nastawy = (los_p, los_i, los_d)

    print("Koniec symulacji")
    
    # przypisanie danych do odczytu
    k_p, k_i, k_d = najlepsze_nastawy
    _, tab_t, tab_v, tab_cel = jeden_przejazd(k_p, k_i, k_d, v_zadana, v_startowa, czas, dt, alfa)

    # Obliczanie max odchylenia
    max_odchylenie = 0
    for v in tab_v:
        diff = abs(v - v_zadana)
        if diff > max_odchylenie:
            max_odchylenie = diff
            
    # Czas stabilizacji
    tolerancja = 0.05 * v_zadana 
    gora = v_zadana + tolerancja
    dol = v_zadana - tolerancja
    
    czas_stabilizacji = None
    ostatni_indeks_poza = 0
    for i in range(len(tab_v)):
        if tab_v[i] > gora or tab_v[i] < dol:
            ostatni_indeks_poza = i
            
    if ostatni_indeks_poza < len(tab_t) - 1:
        czas_stabilizacji = tab_t[ostatni_indeks_poza + 1]
    else:
        czas_stabilizacji = czas

    # Wykresy
    plt.figure(figsize=(12, 7))
    plt.plot(tab_t, tab_cel, 'r--', label=f'Zadana ({v_zadana} km/h)', alpha=0.5)
    plt.plot(tab_t, tab_v, 'b-', label='V auta (BMW M3)', linewidth=2)
    
    plt.axhline(y=gora, color='g', linestyle=':', alpha=0.6, label='Tolerancja +/- 5%')
    plt.axhline(y=dol, color='g', linestyle=':', alpha=0.6)
    
    if czas_stabilizacji:
        plt.axvline(x=czas_stabilizacji, color='k', linestyle='-.', label=f'Czas stab.: {czas_stabilizacji:.1f}s')

    plt.axvspan(czas_symulacji/2, czas_symulacji/1.7, color='yellow', alpha=0.10, label=f'Górka {alfa} stopni')

    plt.title(f'Optymalizacja PID: P={k_p:.2f} I={k_i:.2f} D={k_d:.2f}')
    plt.xlabel('Czas [s]')
    plt.ylabel('Predkosc [km/h]')
    plt.legend(loc='lower right')
    plt.grid(True)
    
    info_text = (
        f"Start V: {v_startowa} km/h\n"
        f"Max odchyłka: {max_odchylenie:.2f} km/h\n"
        f"Czas stabilizacji: {czas_stabilizacji:.2f} s"
    )
    plt.text(0.95, 0.5, info_text, transform=plt.gca().transAxes, 
             bbox=dict(facecolor='white', alpha=0.8), 
             verticalalignment='center', horizontalalignment='right')

    plik = 'wynik.png'
    plt.savefig(plik)
    print(f"Zapisano wykres: {plik}")
    
    koniec_programu = time.time()
    czas_trwania = koniec_programu - start_programu

    v_koniec = tab_v[-1] if tab_v else 0.0
    print(f"\n--- Wyniki ---")
    print(f"Czas obliczeń: {czas_trwania:.4f} s")
    print(f"Max odchylenie: {max_odchylenie:.2f} km/h")
    print(f"Czas stabilizacji: {czas_stabilizacji:.2f} s")
    print(f"Predkosc finalna: {v_koniec:.2f} km/h")

if __name__ == '__main__':
    main()