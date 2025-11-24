/**
 * Andersen.cpp
 * @author kisslune
 */

#include "A5Header.h"

using namespace llvm;
using namespace std;

int main(int argc, char** argv)
{
    // Parse command-line options and build module set
    auto moduleNames =
            OptionBase::parseOptions(argc, argv, "Whole Program Points-to Analysis",
                                     "[options] <input-bitcode...>");

    // Build the SVF module
    SVF::LLVMModuleSet::buildSVFModule(moduleNames);

    SVF::SVFIRBuilder irBuilder;
    auto programAnalysisGraph = irBuilder.build();
    auto constraintGraph = new SVF::ConstraintGraph(programAnalysisGraph);
    constraintGraph->dump("ConstraintGraph");

    Andersen pointerAnalysis(constraintGraph);

    // Run pointer analysis
    pointerAnalysis.runPointerAnalysis();

    // Output the analysis result
    pointerAnalysis.dumpResult();
    SVF::LLVMModuleSet::releaseLLVMModuleSet();
    return 0;
}

void Andersen::runPointerAnalysis()
{
    // Initialize a worklist for analysis
    WorkList<unsigned> analysisWorklist;

    // Initial phase: Handle all address constraints (ptr = &obj)
    // Address constraints indicate an object being directly assigned to a pointer
    // Initialize points-to sets for pointers
    for (auto nodeIter = consg->begin(); nodeIter != consg->end(); ++nodeIter)
    {
        unsigned nodeId = nodeIter->first;
        SVF::ConstraintNode *currentNode = nodeIter->second;

        // Iterate through all address incoming edges for this node and handle address constraints
        const auto &addressEdges = currentNode->getAddrInEdges();
        for (auto *edge : addressEdges)
        {
            SVF::AddrCGEdge *addressEdge = SVF::SVFUtil::dyn_cast<SVF::AddrCGEdge>(edge);
            unsigned objectId = addressEdge->getSrcID();   // Source object
            unsigned pointerId = addressEdge->getDstID();  // Target pointer

            // Add the object to the points-to set of the pointer: object ∈ pts(pointer)
            pts[pointerId].insert(objectId);
            analysisWorklist.push(pointerId);
        }
    }

    // Main loop: Iterate over the constraints until the worklist is empty
    while (!analysisWorklist.empty())
    {
        unsigned pointer = analysisWorklist.pop();
        SVF::ConstraintNode *pointerNode = consg->getConstraintNode(pointer);

        // Handle Store and Load constraints: For each object in the points-to set, add a Copy edge
        for (unsigned object : pts[pointer])
        {
            // Handle Store constraints: *pointer = srcPointer
            // For each srcPointer --Store--> pointer, add srcPointer --Copy--> object
            const auto &storeInEdges = pointerNode->getStoreInEdges();
            for (auto *edge : storeInEdges)
            {
                SVF::StoreCGEdge *storeEdge = SVF::SVFUtil::dyn_cast<SVF::StoreCGEdge>(edge);
                unsigned sourcePointer = storeEdge->getSrcID();

                // Check if the Copy edge (sourcePointer --Copy--> object) already exists
                SVF::ConstraintNode *sourceNode = consg->getConstraintNode(sourcePointer);
                bool hasCopyEdge = false;
                for (auto *copyEdge : sourceNode->getCopyOutEdges())
                {
                    if (copyEdge->getDstID() == object)
                    {
                        hasCopyEdge = true;
                        break;
                    }
                }

                // If the Copy edge doesn't exist, add it and push the source pointer to the worklist
                if (!hasCopyEdge)
                {
                    consg->addCopyCGEdge(sourcePointer, object);
                    analysisWorklist.push(sourcePointer);
                }
            }

            // Handle Load constraints: dstPointer = *pointer
            // For each pointer --Load--> dstPointer, add object --Copy--> dstPointer
            const auto &loadOutEdges = pointerNode->getLoadOutEdges();
            for (auto *edge : loadOutEdges)
            {
                SVF::LoadCGEdge *loadEdge = SVF::SVFUtil::dyn_cast<SVF::LoadCGEdge>(edge);
                unsigned destinationPointer = loadEdge->getDstID();

                // Check if the Copy edge (object --Copy--> dstPointer) already exists
                SVF::ConstraintNode *destinationNode = consg->getConstraintNode(destinationPointer);
                bool hasCopyEdge = false;
                for (auto *copyEdge : destinationNode->getCopyInEdges())
                {
                    if (copyEdge->getSrcID() == object)
                    {
                        hasCopyEdge = true;
                        break;
                    }
                }

                // If the Copy edge doesn't exist, add it and push the object to the worklist
                if (!hasCopyEdge)
                {
                    consg->addCopyCGEdge(object, destinationPointer);
                    analysisWorklist.push(object);
                }
            }
        }

        // Handle Copy constraints: target = pointer
        // Propagate the points-to set of pointer to the target's points-to set
        const auto &copyOutEdges = pointerNode->getCopyOutEdges();
        for (auto *edge : copyOutEdges)
        {
            SVF::CopyCGEdge *copyEdge = SVF::SVFUtil::dyn_cast<SVF::CopyCGEdge>(edge);
            unsigned target = copyEdge->getDstID();

            // Record the previous size of the target's points-to set
            size_t prevSize = pts[target].size();
            // Merge the points-to set of pointer into the target's points-to set
            pts[target].insert(pts[pointer].begin(), pts[pointer].end());

            // If the points-to set has changed, push the target to the worklist
            if (pts[target].size() > prevSize)
            {
                analysisWorklist.push(target);
            }
        }

        // Handle Gep constraints: target = pointer.field
        // Access the field of the pointer's points-to objects
        const auto &gepOutEdges = pointerNode->getGepOutEdges();
        for (auto *edge : gepOutEdges)
        {
            SVF::GepCGEdge *gepEdge = SVF::SVFUtil::dyn_cast<SVF::GepCGEdge>(edge);
            unsigned target = gepEdge->getDstID();

            // Record the previous size of the target's points-to set
            size_t prevSize = pts[target].size();
            // For each object in the pointer's points-to set, access the corresponding field object
            for (unsigned object : pts[pointer])
            {
                unsigned fieldObject = consg->getGepObjVar(object, gepEdge);
                pts[target].insert(fieldObject);
            }

            // If the points-to set has changed, push the target to the worklist
            if (pts[target].size() > prevSize)
            {
                analysisWorklist.push(target);
            }
        }
    }
}
