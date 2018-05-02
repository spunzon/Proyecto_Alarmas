/*
 * lista.c
 *
 *  Created on: 22 abr. 2018
 *      Author: spunz
 */
#include <stdio.h>  /* printf (), scanf (), puts () */
#include <stdlib.h> /* malloc (), free (), exit () */
#include <string.h> /* strchr () */
#include "lista.h"

/*
  La primera operación que hay que hacer siempre con una lista es inicializarla,
  esto consiste en asignar NULL al puntero que apunta a la cabeza de la lista
*/

void empezar_lista (palarm *plist) {
  *plist = NULL;
}


void crear_nodo_lista (palarm *pl) {
  if ((*pl = (palarm) malloc (sizeof (alarm))) == NULL) {
      printf ("\nERROR: Memoria insuficiente.");
      exit (1);
    }
}


void aniadir_a_lista (palarm *plist, uint8_t horas, uint8_t minutos, uint8_t segundos) {
  palarm p;

  crear_nodo_lista (&p);
  ((p)->tiempo.Hours) = horas;
  ((p)->tiempo.Minutes) = minutos;
  ((p)->tiempo.Seconds) = segundos;
  ((p)->psiguiente) = *plist;
  *plist = p;
}


void liberar_nodo_lista (palarm *pl) {
  free (*pl);
}


void terminar_lista (palarm *plist) {
  palarm p;

  p = *plist;
  while (p) {       // recorre toda la lista liberando cada nodo
    *plist = psig (p);
    liberar_nodo_lista (&p);
    p = *plist;
  }
}


BOOLEAN quitar_de_lista (palarm *plist, int horas, int minutos, int segundos) {
  palarm p = *plist, pant = NULL;
  BOOLEAN elemento_quitado = FALSE;

  while (p != NULL && ((p)->tiempo.Hours) != horas && ((p)->tiempo.Minutes) != minutos && ((p)->tiempo.Seconds) != segundos) {    // busca elemento
    pant = p;
    p = ((p)->psiguiente);
  }

  if (p!=NULL) {        // encontrado
    if (pant==NULL) {   // primero de la lista
      *plist = ((p)->psiguiente);
      liberar_nodo_lista (&p);
    } else {
      *plist = ((pant)->psiguiente);
      liberar_nodo_lista (&pant);
    }
    elemento_quitado = TRUE;
  }
  return (elemento_quitado);
}

void listar_lista (palarm plist)
{
  printf ("\n\nElementos en lista: ");
  if (plist == NULL)
    printf ("(ninguno)");
  else {
    palarm p;
    for (p = plist; p != NULL; p = ((p)->psiguiente)) // recorre todos los nodos
      printf ("%d:%d:%d ", ((p)->tiempo.Hours),((p)->tiempo.Minutes),((p)->tiempo.Seconds));
  }
  puts (""); // esta sentencia es equivalente a putch ('\n') y printf ("\n")
}
