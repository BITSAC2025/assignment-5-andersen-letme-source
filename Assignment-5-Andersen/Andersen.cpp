/**
 * Andersen.cpp
 * @author kisslune
 */

/**
 * Andersen.cpp
 * 基于包含约束(Andersen-style)的集合约束指针分析实现。
 * 约束：
 *   1) x = &a        : PT(x)  ⊇ {a}
 *   2) x = y         : PT(x)  ⊇ PT(y)
 *   3) *p = q        : ∀o∈PT(p):  PT(o) ⊇ PT(q)
 *   4) x = *p        : ∀o∈PT(p):  PT(x) ⊇ PT(o)
 *
 */

#include "A5Header.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <deque>

using namespace llvm;
using namespace std;

int main(int argc, char** argv)
{
    auto moduleNameVec =
            OptionBase::parseOptions(argc, argv, "Whole Program Points-to Analysis",
                                     "[options] <input-bitcode...>");

    SVF::LLVMModuleSet::buildSVFModule(moduleNameVec);

    SVF::SVFIRBuilder builder;
    auto pag = builder.build();

    // 构建约束图（本实现不直接依赖其 API；但课堂给了它的构建）
    auto consg = new SVF::ConstraintGraph(pag);

    // consg->dump();

    Andersen andersen(consg);
    andersen.runPointerAnalysis();
    andersen.dumpResult();

    SVF::LLVMModuleSet::releaseLLVMModuleSet();
    return 0;
}


void Andersen::runPointerAnalysis()
{
    using NodeID = unsigned;

    // ----------------------------------------------------
    // 1) 从 SVFIR(PAG) 收集基本约束边
    // ----------------------------------------------------
    SVF::SVFIR* pag = SVF::PAG::getPAG();

    struct SimpleEdge { NodeID src; NodeID dst; };
    std::vector<SimpleEdge> addrEdges;   // x --Addr--> a           (x = &a)
    std::vector<SimpleEdge> copyEdges;   // y --Copy--> x           (x = y)
    std::vector<SimpleEdge> loadEdges;   // p --Load--> x           (x = *p)
    std::vector<SimpleEdge> storeEdges;  // q --Store--> p          (*p = q)

    // 基本 Addr/Copy
    for (SVF::PAGEdge *edge : pag->getSVFStmtSet(SVF::PAGEdge::Addr)) {
        addrEdges.push_back({edge->getSrcID(), edge->getDstID()});
    }
    for (SVF::PAGEdge *edge : pag->getSVFStmtSet(SVF::PAGEdge::Copy)) {
        copyEdges.push_back({edge->getSrcID(), edge->getDstID()});
    }

    // Phi, Select, Call, Ret, ThreadFork/Join 视为 Copy（流不敏感、全局分析）
    for (SVF::PAGEdge *edge : pag->getSVFStmtSet(SVF::PAGEdge::Phi)) {
        const SVF::PhiStmt *phi = SVF::SVFUtil::cast<SVF::PhiStmt>(edge);
        for (const auto opVar : phi->getOpndVars()) {
            copyEdges.push_back({opVar->getId(), phi->getResID()});
        }
    }
    for (SVF::PAGEdge *edge : pag->getSVFStmtSet(SVF::PAGEdge::Select)) {
        const SVF::SelectStmt *sel = SVF::SVFUtil::cast<SVF::SelectStmt>(edge);
        for (const auto opVar : sel->getOpndVars()) {
            copyEdges.push_back({opVar->getId(), sel->getResID()});
        }
    }
    for (SVF::PAGEdge *edge : pag->getSVFStmtSet(SVF::PAGEdge::Call)) {
        copyEdges.push_back({edge->getSrcID(), edge->getDstID()});
    }
    for (SVF::PAGEdge *edge : pag->getSVFStmtSet(SVF::PAGEdge::Ret)) {
        copyEdges.push_back({edge->getSrcID(), edge->getDstID()});
    }
    for (SVF::PAGEdge *edge : pag->getSVFStmtSet(SVF::PAGEdge::ThreadFork)) {
        copyEdges.push_back({edge->getSrcID(), edge->getDstID()});
    }
    for (SVF::PAGEdge *edge : pag->getSVFStmtSet(SVF::PAGEdge::ThreadJoin)) {
        copyEdges.push_back({edge->getSrcID(), edge->getDstID()});
    }

    // Load / Store
    for (SVF::PAGEdge *edge : pag->getSVFStmtSet(SVF::PAGEdge::Load)) {
        loadEdges.push_back({edge->getSrcID(), edge->getDstID()});  // p -> x
    }
    for (SVF::PAGEdge *edge : pag->getSVFStmtSet(SVF::PAGEdge::Store)) {
        storeEdges.push_back({edge->getSrcID(), edge->getDstID()}); // q -> p
    }

    // ----------------------------------------------------
    // 2) 构建辅助结构：复制后继、load/store 基指针映射
    // ----------------------------------------------------
    // pts[v] 已在类中定义：map<unsigned, set<unsigned>>
    std::unordered_map<NodeID, std::vector<NodeID>> copySucc;      // v 的复制后继
    std::unordered_map<NodeID, std::unordered_set<NodeID>> copySuccSet; // 去重
    std::unordered_map<NodeID, std::vector<NodeID>> loadBase;      // p -> [x...] (x = *p)
    std::unordered_map<NodeID, std::vector<NodeID>> storeBase;     // p -> [q...] (*p = q)

    for (const auto &e : copyEdges) {
        copySucc[e.src].push_back(e.dst);
        copySuccSet[e.src].insert(e.dst);
    }
    for (const auto &e : loadEdges) {
        loadBase[e.src].push_back(e.dst);
    }
    for (const auto &e : storeEdges) {
        storeBase[e.dst].push_back(e.src);   // 记录到 p：谁往 *p 里存
    }

    // 记录 (p,o) 已处理，避免针对同一 o∈PT(p) 反复生成间接复制边
    std::unordered_map<NodeID, std::unordered_set<NodeID>> processedPO;

    // ----------------------------------------------------
    // 3) 工作队列固定点
    // ----------------------------------------------------
    std::deque<NodeID> wl;
    std::unordered_set<NodeID> inWL;

    auto enqueue = [&](NodeID n) {
        if (!inWL.count(n)) { wl.push_back(n); inWL.insert(n); }
    };

    auto addPts = [&](NodeID v, NodeID o) -> bool {
        std::set<NodeID> &S = pts[v];
        if (S.insert(o).second) { enqueue(v); return true; }
        return false;
    };

    // 初始化：Addr 约束
    for (const auto &e : addrEdges) {
        addPts(e.src, e.dst);  // PT(x) ⊇ {a}
    }

    // 主循环
    while (!wl.empty()) {
        NodeID v = wl.front(); wl.pop_front(); inWL.erase(v);

        // (a) 复制传播：PT(succ) ⊇ PT(v)
        auto itSucc = copySucc.find(v);
        if (itSucc != copySucc.end()) {
            for (NodeID w : itSucc->second) {
                for (NodeID o : pts[v]) addPts(w, o);
            }
        }

        // (b) 针对每个 o∈PT(v) 处理由 load/store 诱导的间接复制
        for (NodeID o : pts[v]) {
            if (processedPO[v].count(o)) continue;
            processedPO[v].insert(o);

            // Load：v --Load--> x  ⇒  o --Copy--> x  ⇒ PT(x) ⊇ PT(o)
            auto itL = loadBase.find(v);
            if (itL != loadBase.end()) {
                for (NodeID x : itL->second) {
                    if (!copySuccSet[o].count(x)) {
                        copySucc[o].push_back(x);
                        copySuccSet[o].insert(x);
                    }
                    for (NodeID oo : pts[o]) addPts(x, oo);
                }
            }

            // Store：q --Store--> v  ⇒  q --Copy--> o  ⇒ PT(o) ⊇ PT(q)
            auto itS = storeBase.find(v);
            if (itS != storeBase.end()) {
                for (NodeID qNode : itS->second) {
                    if (!copySuccSet[qNode].count(o)) {
                        copySucc[qNode].push_back(o);
                        copySuccSet[qNode].insert(o);
                    }
                    for (NodeID qq : pts[qNode]) addPts(o, qq);
                }
            }
        }
    }
}
