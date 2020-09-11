//
//  Graphe.cpp
//  Classe pour graphes orientés pondérés (non négativement) avec listes d'adjacence
//
//  Mario Marchand automne 2016.
//

#include "graphe.h"
#include <set>
#include <queue>

using namespace std;

//! \brief Constructeur avec paramètre du nombre de sommets désiré
//! \param[in] p_nbSommets indique le nombre de sommets désiré
//! \post crée le vecteur de p_nbSommets de listes d'adjacence vides
Graphe::Graphe(size_t p_nbSommets)
    : m_listesAdj(p_nbSommets), nbArcs(0)
{
}

//! \brief change le nombre de sommets du graphe
//! \param[in] p_nouvelleTaille indique le nouveau nombre de sommet
//! \post le graphe est un vecteur de p_nouvelleTaille de listes d'adjacence
//! \post les anciennes listes d'adjacence sont toujours présentes lorsque p_nouvelleTaille >= à l'ancienne taille
//! \post les dernières listes d'adjacence sont enlevées lorsque p_nouvelleTaille < à l'ancienne taille
void Graphe::resize(size_t p_nouvelleTaille)
{
    m_listesAdj.resize(p_nouvelleTaille);
}

size_t Graphe::getNbSommets() const
{
	return m_listesAdj.size();
}

size_t Graphe::getNbArcs() const
{
    return nbArcs;
}

//! \brief ajoute un arc d'un poids donné dans le graphe
//! \param[in] i: le sommet origine de l'arc
//! \param[in] j: le sommet destination de l'arc
//! \param[in] poids: le poids de l'arc
//! \pre les sommets i et j doivent exister
//! \throws logic_error lorsque le sommet i ou le sommet j n'existe pas
//! \throws logic_error lorsque le poids == numeric_limits<unsigned int>::max()
void Graphe::ajouterArc(size_t i, size_t j, unsigned int poids)
{
    if (i >= m_listesAdj.size())
        throw logic_error("Graphe::ajouterArc(): tentative d'ajouter l'arc(i,j) avec un sommet i inexistant");
    if (j >= m_listesAdj.size())
        throw logic_error("Graphe::ajouterArc(): tentative d'ajouter l'arc(i,j) avec un sommet j inexistant");
    if (poids == numeric_limits<unsigned int>::max())
        throw logic_error("Graphe::ajouterArc(): valeur de poids interdite");
    m_listesAdj[i].emplace_back(Arc(j, poids));
    ++nbArcs;
}

//! \brief enlève un arc dans le graphe
//! \param[in] i: le sommet origine de l'arc
//! \param[in] j: le sommet destination de l'arc
//! \pre l'arc (i,j) et les sommets i et j dovent exister
//! \post enlève l'arc mais n'enlève jamais le sommet i
//! \throws logic_error lorsque le sommet i ou le sommet j n'existe pas
//! \throws logic_error lorsque l'arc n'existe pas
void Graphe::enleverArc(size_t i, size_t j)
{
    if (i >= m_listesAdj.size())
        throw logic_error("Graphe::enleverArc(): tentative d'enlever l'arc(i,j) avec un sommet i inexistant");
    if (j >= m_listesAdj.size())
        throw logic_error("Graphe::enleverArc(): tentative d'enlever l'arc(i,j) avec un sommet j inexistant");
    auto &liste = m_listesAdj[i];
    bool arc_enleve = false;
    for (auto itr = liste.end(); itr != liste.begin();) //on débute par la fin par choix
    {
        if ((--itr)->destination == j)
        {
            liste.erase(itr);
            arc_enleve = true;
            break;
        }
    }
    if (!arc_enleve)
        throw logic_error("Graphe::enleverArc: cet arc n'existe pas; donc impossible de l'enlever");
    --nbArcs;
}


unsigned int Graphe::getPoids(size_t i, size_t j) const
{
    if (i >= m_listesAdj.size()) throw logic_error("Graphe::getPoids(): l'incice i n,est pas un sommet existant");
    for (auto & arc : m_listesAdj[i])
    {
        if (arc.destination == j) return arc.poids;
    }
    throw logic_error("Graphe::getPoids(): l'arc(i,j) est inexistant");
}

////! \brief Algorithme de Dijkstra permettant de trouver le plus court chemin entre p_origine et p_destination
////! \pre p_origine et p_destination doivent être des sommets du graphe
////! \return la longueur du plus court chemin est retournée
////! \param[out] le chemin est retourné (un seul noeud si p_destination == p_origine ou si p_destination est inatteignable)
////! \return la longueur du chemin (= numeric_limits<unsigned int>::max() si p_destination n'est pas atteignable)
////! \throws logic_error lorsque p_origine ou p_destination n'existe pas
unsigned int Graphe::plusCourtChemin(size_t p_origine, size_t p_destination, std::vector<size_t> &p_chemin) const
{
    if (p_origine >= m_listesAdj.size() || p_destination >= m_listesAdj.size())
        throw logic_error("Graphe::dijkstra(): p_origine ou p_destination n'existe pas");

    p_chemin.clear();

    if (p_origine == p_destination)
    {
        p_chemin.push_back(p_destination);
        return 0;
    }

    std::priority_queue< pair<unsigned int, size_t>, vector<pair<unsigned int, size_t>>, less<pair<unsigned int, size_t>> > heap;
    heap.push(make_pair(0, p_origine));

    vector<unsigned int> dist(m_listesAdj.size(), numeric_limits<unsigned int>::max());
    dist[p_origine] = 0;

    vector<size_t> prev(m_listesAdj.size(), numeric_limits<size_t>::max());

    while(!heap.empty()) {
        size_t u = heap.top().second;
        heap.pop();

        if(u == p_destination)
            break;

        for (auto u_itr = m_listesAdj[u].begin(); u_itr != m_listesAdj[u].end(); ++u_itr)
        {
            size_t v = u_itr->destination;
            int w = u_itr->poids;

            if(dist[v] > dist[u] + w) {
                dist[v] = dist[u] + w;
                prev[v] = u;
                heap.push(make_pair(dist[v], v));
            }
        }
    }

    //On a pas de solution
    if (prev[p_destination] == numeric_limits<unsigned int>::max())
    {
        p_chemin.push_back(p_destination);
        return numeric_limits<unsigned int>::max();
    }

//    queue<size_t> path;
//    size_t vertex = p_destination;
//    path.push(p_origine);
//
//    while (prev[vertex] != p_origine)
//    {
//        vertex = prev[vertex];
//        path.push(vertex);
//    }
//
//    while (!path.empty())
//    {
//        size_t temp = path.front();
//        p_chemin.push_back(temp);
//        path.pop();
//    }

//p_chemin.push_back(p_destination);

        //On a une solution, donc construire le plus court chemin à l'aide de predecesseur[]
    stack<size_t> pileDuChemin;
    size_t numero = p_destination;
    pileDuChemin.push(numero);
    while (prev[numero] != numeric_limits<size_t>::max())
    {
        numero = prev[numero];
        pileDuChemin.push(numero);
    }
    while (!pileDuChemin.empty())
    {
        size_t temp = pileDuChemin.top();
        p_chemin.push_back(temp);
        pileDuChemin.pop();
    }

    return dist[p_destination];
}

////! \brief Algorithme de Dijkstra permettant de trouver le plus court chemin entre p_origine et p_destination
////! \pre p_origine et p_destination doivent être des sommets du graphe
////! \return la longueur du plus court chemin est retournée
////! \param[out] le chemin est retourné (un seul noeud si p_destination == p_origine ou si p_destination est inatteignable)
////! \return la longueur du chemin (= numeric_limits<unsigned int>::max() si p_destination n'est pas atteignable)
////! \throws logic_error lorsque p_origine ou p_destination n'existe pas
//unsigned int Graphe::plusCourtChemin(size_t p_origine, size_t p_destination, std::vector<size_t> &p_chemin) const
//{
//    if (p_origine >= m_listesAdj.size() || p_destination >= m_listesAdj.size())
//        throw logic_error("Graphe::dijkstra(): p_origine ou p_destination n'existe pas");
//
//    p_chemin.clear();
//
//    if (p_origine == p_destination)
//    {
//        p_chemin.push_back(p_destination);
//        return 0;
//    }
//
//    //Vecteur des distances entre le point d'origine et un noeud donné
//    //m_listeAdj.size() -> Nombre d'éléments initialisés
//    //numeric_limits<unsigned int>::max() -> Valeur d'initialisation
//    vector<unsigned int> distance(m_listesAdj.size(), numeric_limits<unsigned int>::max());
//    distance[p_origine] = 0;
//
//    //Vecteur des index des noeuds précédents un noeud donné
//    //m_listeAdj.size() -> Nombre d'éléments initialisés
//    //numeric_limits<size_t>::max() -> Valeur d'initialisation
//    vector<size_t> predecesseur(m_listesAdj.size(), numeric_limits<size_t>::max());
//
//    //Noeuds non solutionnés
//    list<size_t> q;
//
//    //TODO: Créer une liste des noeuds solutionnés
//
//    //Construction de la liste des noeuds non solutionnés
//    //TODO: Investiguer la nécessité de la boucle
//    for (size_t i = 0; i < m_listesAdj.size(); ++i)
//    {
//        q.push_back(i);
//    }
//
//    //Tant que l'on a pas solutionné tous les noeuds
//    while (!q.empty())
//    {
//        //trouver uStar dans q tel que distance[uStar] est minimal
//        list<size_t>::iterator uStar_itr = q.end();
//
//        //À l'origine, la distance minimale est infinie
//        unsigned int min = numeric_limits<unsigned int>::max();
//
//        //Pour tous les noeuds non solutionnés, on cherche le noeud adjacent au point
//        //d'origine dont la distance est la plus petite
//        for (auto itr = q.begin(); itr != q.end(); ++itr)
//        {
//            if (distance[*itr] < min)
//            {
//                min = distance[*itr];
//                uStar_itr = itr;
//            }
//        }
//
//        //Quitter la boucle : il est impossible de se rendre à destination
//        if (uStar_itr == q.end()) break;
//
//        //Noeud solutionné (dont la distance avec la point d'origine est la plus petite)
//        size_t uStar = *uStar_itr;
//
//        //On enlève le noeud solutionné de la liste des noeuds non solutionnés
//        q.erase(uStar_itr); //l'enlevé de q
//
//        //Quitter la boucle: on a obtenu distance[p_destination] et predecesseur[p_destination]
//        if (uStar == p_destination) break;
//
//        //Relâcher les arcs sortant de uStar
//        for (auto u_itr = m_listesAdj[uStar].begin(); u_itr != m_listesAdj[uStar].end(); ++u_itr)
//        {
//            //TODO: Vérifier que u_itr n'est pas dans la liste des noeuds solutionnés
//            unsigned int temp = distance[uStar] + u_itr->poids;
//
//            if (temp < distance[u_itr->destination])
//            {
//                distance[u_itr->destination] = temp;
//                predecesseur[u_itr->destination] = uStar;
//            }
//        }
//    }
//
//    //On a pas de solution
//    if (predecesseur[p_destination] == numeric_limits<unsigned int>::max())
//    {
//        p_chemin.push_back(p_destination);
//        return numeric_limits<unsigned int>::max();
//    }
//
//    //On a une solution, donc construire le plus court chemin à l'aide de predecesseur[]
//    stack<size_t> pileDuChemin;
//    size_t numero = p_destination;
//    pileDuChemin.push(numero);
//    while (predecesseur[numero] != numeric_limits<size_t>::max())
//    {
//        numero = predecesseur[numero];
//        pileDuChemin.push(numero);
//    }
//    while (!pileDuChemin.empty())
//    {
//        size_t temp = pileDuChemin.top();
//        p_chemin.push_back(temp);
//        pileDuChemin.pop();
//    }
//
//    return distance[p_destination];
//}

