> [!warning] Ersatt
> Första versionen av stackbeskrivningen. Den aktuella är [[StackV2]]. Vad som faktiskt finns i repot står i [[Codebase map]].

För ett AI-projekt med en svärm av drönare är det lämpligt att dela upp systemet i flera lager där varje komponent har ett tydligt ansvar. Kombinationen av **Ubuntu/Linux, Docker, ROS 2, PX4, Gazebo och Python/C++** gör det möjligt att fokusera på AI och svärmintelligens utan att behöva implementera hela flygsystemet själva.

  

**Ubuntu/Linux – gemensam utvecklingsmiljö:** Ubuntu används som den gemensamma miljön eftersom ROS 2, PX4 och Gazebo har starkt stöd för Linux. Eftersom gruppen använder Windows, macOS och Linux kan Windows-användare exempelvis använda WSL2 och macOS-användare en virtuell Linux-miljö. Detta minskar skillnader mellan utvecklingsmiljöerna och gör projektet enklare att reproducera.

  

**Docker – gemensam och reproducerbar miljö:** Docker används för att paketera projektets mjukvara och alla nödvändiga beroenden i containers. Detta är särskilt användbart eftersom gruppmedlemmarna använder olika operativsystem. Genom en gemensam Docker-konfiguration kan alla använda samma versioner av exempelvis ROS 2, Python och projektets bibliotek. Det gör det även enklare för en ny gruppmedlem att sätta upp projektet och minskar risken för problem där koden fungerar på en dator men inte på en annan. Docker bör främst ses som infrastruktur för projektet och inte som en del av själva drönarnas funktionalitet.

  

**ROS 2 – kommunikation mellan systemets delar:** ROS 2 fungerar som kommunikationslagret mellan drönarna, AI-algoritmerna, sensorerna och flygkontrollern. Genom topics, services och actions kan olika delar av systemet utvecklas separat men ändå kommunicera med varandra. Detta passar särskilt bra för en svärm eftersom varje drönare kan representeras av separata ROS 2-noder och utbyta information med både andra drönare och det centrala AI-systemet.

  

**PX4 – autopilot och flygkontroll:** PX4 ansvarar för den lågnivåstyrning som krävs för att faktiskt flyga en drönare, exempelvis stabilisering, positionskontroll, sensorhantering och motorstyrning. Detta är viktigt eftersom gruppen då slipper utveckla ett komplett flygkontrollsystem själva. AI-systemet kan istället ge drönaren kommandon på en högre nivå, medan PX4 hanterar hur dessa kommandon genomförs. PX4 gör även lösningen mer realistisk eftersom samma typ av flygkontroller används i verkliga autonoma drönarsystem.

  

**Gazebo – simulering:** Gazebo används för att skapa den virtuella världen där drönarna flyger. Det simulerar bland annat fysik, sensorer, kollisioner och flera drönare samtidigt. På så sätt kan algoritmerna testas säkert och upprepade gånger utan fysisk hårdvara. Det gör det även möjligt att skapa olika miljöer och scenarier för att undersöka hur svärmen reagerar på förändrade förhållanden.

  

**Python – AI och högnivålogik:** Python lämpar sig väl för projektets AI-del eftersom det finns ett stort ekosystem av bibliotek som PyTorch, NumPy, OpenCV och scikit-learn. Dessa kan användas för exempelvis maskininlärning, bildbehandling, numeriska beräkningar och beslutsfattande. Python gör det också snabbt att experimentera med olika algoritmer för svärmbeteende, vilket är fördelaktigt i ett projekt där olika AI-metoder kan behöva testas och jämföras.

  

**C++ – prestandakritiska delar:** C++ kan användas där hög prestanda eller låg fördröjning är viktigt, exempelvis vid frekventa kontrollloopar, stora simuleringar eller beräkningstunga ROS 2-noder. C++ är också ett centralt språk inom ROS 2 och används av många av de underliggande robotikkomponenterna. Det behöver dock inte innebära att hela projektet skrivs i C++. Python- och C++-noder kan kommunicera via ROS 2, vilket gör det möjligt att välja språk efter behov.

  

**AI-/svärmlagret – projektets huvudsakliga fokus:** Ovanpå dessa komponenter implementeras själva svärmintelligensen. Det kan exempelvis handla om uppgiftsfördelning mellan drönare, formationsflygning, kollisionsundvikande, samordnad utforskning, banplanering, kommunikation mellan drönare eller reinforcement learning. Detta är projektets huvudsakliga AI-del och är där gruppen kan undersöka hur flera autonoma drönare kan samarbeta och fatta beslut som en svärm.

  

**Övergripande arkitektur:**

  

```text

                 AI / Svärmintelligens

                    Python / C++

                          │

                        ROS 2

              kommunikation & koordinering

                          │

                    ┌─────┴─────┐

                    │   Docker  │

                    │  Miljö    │

                    └─────┬─────┘

                          │

              ┌───────────┴───────────┐

              ▼                       ▼

            PX4                     Gazebo

       flygkontroll              simulering

              │                       │

              ├──── Drönare 1 ───────┤

              ├──── Drönare 2 ───────┤

              └──── Drönare 3 ───────┘

```

  

Den stora fördelen med denna arkitektur är **separationen av ansvar**: Docker skapar en reproducerbar utvecklingsmiljö, ROS 2 kopplar ihop systemets olika delar, Gazebo simulerar världen, PX4 sköter flygkontrollen och Python/C++ används för implementationen av AI och övrig logik. På så sätt kan projektets fokus ligga på **svärmintelligens och autonomt samarbete**, snarare än på att bygga ett komplett flyg- och simuleringssystem från grunden.