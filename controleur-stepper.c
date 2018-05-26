/* 
 * Gère le déplacement d'un moteur pas à pas (stepper) au moyen de
 * deux boutons connectés aux entrées INT0 et INT1.
 * Un circuit intégré spécifique aide le micro-contrôleur à générer
 * la commutation.
 * @author jean-michel-gonet
 */

#include <xc.h>

/**
 * Configure le ECCP3 et le port A pour produire la commutation
 * de stationnement sur le pas en cours.
 * @param pas Position en cours dans la séquence de stationnement.
 */
void commutationStationnement(unsigned char pas) {
    // Séquence de commutation pour le stationnement.
    static unsigned char commutateursStationnement[] = {
        1, 4, 2, 8
    };

    unsigned char n;

    // En arrêt, le PWM est à 50%:
    CCPR3L = 16;

    // Les 2 bits plus signifiants du numéro de séquence contiennent
    // la position des commutateurs pour les ponts:
    n = pas >> 3;
    PORTA = commutateursStationnement[n];
}

/**
 * Configure le ECCP3 et le port A pour produire le micro-pas correspondant
 * sur la séquence de commutation.
 * @param pas Position en cours dans la séquence de commutation.
 */
void commutationDeplacement(char pas) {
    // Valeurs pré-calculées pour les micro-pas.
    static unsigned char cos[] = {
        32, 31, 27, 22, 16, 10,  5,  1,
         0,  1,  5, 10, 16, 22, 27, 31
    };

    // Séquence de commutation pour le déplacement.
    static unsigned char commutateursDeplacement[] = {
        5, 6, 10, 9
    };

    char n;

    // Les 4 bits moins signifiants du numéro de séquence contiennent
    // l'index du tableau de micro-pas:
    n = pas & 0x0F;
    CCPR3L = cos[n];

    // Les 2 bits plus signifiants du numéro de séquence contiennent
    // la position des commutateurs pour les ponts:
    n = pas >> 3;
    PORTA = commutateursDeplacement[n];
}

/**
 * Liste d'états pour la machine à états.
 */
enum Etat {
    /** Le moteur est à l'arrêt, sur un pas entier.*/
    ARRET,
    /** Le moteur avance. */
    MARCHE_AVANT,
    /** Le moteur va s'arrêter dès qu'il atteint un pas complet.*/
    FREIN_AVANT,
    /** Le moteur recule. */
    MARCHE_ARRIERE,
    /** Le moteur va s'arrêter dès qu'il atteint un pas complet.*/
    FREIN_ARRIERE
};

/**
 * Liste d'événements pour la machine à états.
 */
enum Evenement {
    /** Le moteur doit avancer.*/
    AVANCE,
    /** Le moteur doit reculer.*/
    RECULE,
    /** Le moteur doit s'arrêter.*/
    ARRETE,
    /** Suivante �tape dans la séquence.*/
    TICTAC
};

/**
 * Machine à états.
 * @param evenement L'événement à gérer.
 */
void machine(enum Evenement evenement) {
    // État principal de la machine.
    static enum Etat etat = ARRET;

    // Position dans la séquence de commutation, entre 0 et 31.
    static unsigned char pas = 0;

    switch(etat) {
        case ARRET:
            switch(evenement) {
                case AVANCE:
                    etat = MARCHE_AVANT;
                    break;
                case RECULE:
                    etat = MARCHE_ARRIERE;
                    break;
            }
            break;

        // Le moteur avance en suivant la séquence de commutation
        // jusqu'à ce qu'il reçoive l'ordre de s'arrêter.
        case MARCHE_AVANT:
            switch(evenement) {
                case TICTAC:
                    commutationDeplacement(pas);
                    pas++;
                    if (pas > 31) {
                        pas = 0;
                    }
                    break;
                case RECULE:
                case ARRETE:
                    etat = FREIN_AVANT;
                    break;
            }
            break;

        // Le moteur continue d'avancer jusqu'à ce qu'il
        // arrive sur un pas entier.
        case FREIN_AVANT:
            switch(evenement) {
                case TICTAC:
                    switch(pas) {
                        case 0:
                        case 8:
                        case 16:
                        case 24:
                            commutationStationnement(pas);
                            etat = ARRET;
                            break;
                        default:
                            commutationDeplacement(pas);
                            pas++;
                            if (pas > 31) {
                                pas = 0;
                            }
                            break;
                    }
                    break;
            }
            break;
        case MARCHE_ARRIERE:
            switch(evenement) {
                case TICTAC:
                    commutationDeplacement(pas);
                    pas--;
                    if (pas > 31) {
                        pas = 31;
                    }
                    break;
                case AVANCE:
                case ARRETE:
                    etat = FREIN_ARRIERE;
                    break;
            }
            break;

        // Le moteur continue de reculer jusqu'à ce qu'il
        // arrive sur un pas entier.
        case FREIN_ARRIERE:
            switch(evenement) {
                case TICTAC:
                    switch(pas) {
                        case 0:
                        case 8:
                        case 16:
                        case 24:
                            commutationStationnement(pas);
                            etat = ARRET;
                            break;
                        default:
                            commutationDeplacement(pas);
                            pas--;
                            if (pas > 31) {
                                pas = 31;
                            }
                            break;
                    }
                    break;
            }
            break;
    }
}

/**
 * Interruptions.
 */
void interrupt interruptionsHP() {
    
    static char n = 0;

    // Détecte de quel type d'interruption il s'agit:
    if (PIR1bits.TMR2IF) {
        PIR1bits.TMR2IF = 0;
        if (n == 0) {
            machine(TICTAC);
        }
        n++;
        if (n > 25) {
            n = 0;
        }
    }

    // Détecte de quel type d'interruption il s'agit:
    if (INTCON3bits.INT2IF) {
        INTCON3bits.INT2IF=0;
        machine(AVANCE);
    }
    if (INTCON3bits.INT1IF) {
        INTCON3bits.INT1IF = 0;
        machine(RECULE);
    }
}

/**
 * Point d'entrée du programme.
 * Configure le port A comme sortie, le temporisateur 2, le module
 * CCP3 et les interruptions INT0 et INT1.
 */
void main() {
    ANSELA = 0x00;      // Désactive les convertisseurs A/D.
    ANSELB = 0x00;      // Désactive les convertisseurs A/D.
    ANSELC = 0x00;      // Désactive les convertisseurs A/D.

    TRISA = 0x00;       // Tous les bits du port A comme sorties.


    // Active le PWM sur CCP5:
    T2CONbits.T2CKPS = 1;       // Pas de diviseur de fréq. en entrée.
    T2CONbits.T2OUTPS = 8;      // Pour ménager le traitement d'int.
    PR2 = 32;                   // Période du tmr2: 32
    T2CONbits.TMR2ON = 1;       // Active le tmr2
    CCPTMRS0bits.C3TSEL = 0;    // CCP3 branché sur tmr2
    CCP3CONbits.P3M = 2;        // Mode demi-pont.
    CCP3CONbits.CCP3M = 0xC;    // Active le CCP3.

    TRISBbits.RB5 = 0;          // Active la sortie P3A.
    TRISCbits.RC7 = 0;          // Active la sortie P3B.

    PORTA = 9;
    PORTB = 0x00;
    PORTC = 0xFF;

    // Prépare les interruptions de haute priorité temporisateur 2:
    PIE1bits.TMR2IE = 1;        // Active les interruptions.
    IPR1bits.TMR2IP = 1;        // En haute priorité.
    PIR1bits.TMR2IF = 0;        // Baisse le drapeau.

    // Prépare les interruptions de basse priorité INT1 et INT2:
    TRISBbits.RB2 = 1;          // INT2 comme entrée digitale.
    TRISBbits.RB1 = 1;          // INT1 comme entrée digitale.

    INTCON2bits.RBPU=0;         // Active les résistances de tirage...
    WPUBbits.WPUB2 = 1;         // ... sur INT2 ...
    WPUBbits.WPUB1 = 1;         // ... et INT1.

    INTCON2bits.INTEDG2 = 0;    // Int. de INT2 sur flanc descendant.
    INTCON2bits.INTEDG1 = 0;    // Int. de INT1 sur flanc descendant.

    INTCON3bits.INT2IE = 1;     // Interruptions pour INT2...
    INTCON3bits.INT2IP = 1;     // ... en basse priorité.
    INTCON3bits.INT1IE = 1;     // Interruptions pour INT1...
    INTCON3bits.INT1IP = 1;     // ... en basse priorité.

    // Active les interruptions de haute priorité:
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 0;

    // Place le moteur en position arrêtée sur le pas 0:
    commutationStationnement(0);

    // Dodo:
    while(1);
}
