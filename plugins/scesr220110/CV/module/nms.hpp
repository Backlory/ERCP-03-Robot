#pragma once
#include <torch/torch.h>
#include <torchvision/ops/ops.h>
#include "utils.hpp"

using namespace torch::indexing;

namespace nanodet {

    /// <summary>
    /// Performs non-maximum suppression in a batched fashion.
    /// Modified from https://github.com/pytorch/vision/blob/505cd6957711af790211896d32b40291bea1bc21/torchvision/ops/boxes.py#L39.
    /// In order to perform NMS independently per class, we add an offset to all
    /// the boxes. The offset is dependent only on the class idx, and is large
    /// enough so that boxes from different classes do not overlap.
    /// </summary>
    /// <param name="boxes">boxes in shape (N, 4).</param>
    /// <param name="scores">scores in shape (N, ).</param>
    /// <param name="rows">each index value correspond to a bbox cluster,
    ///     and NMS will not be applied between elements of different idxs,
    ///     shape (N, ).</param>
    /// <param name="nms_cfg"> specify nms type and other parameters like iou_thr.
    ///     Possible keys includes the following.
    ///     - iou_thr (float): IoU threshold used for NMS.
    ///     - split_thr (float): threshold number of boxes. In some cases the
    ///         number of boxes is large (e.g., 200k). To avoid OOM during
    ///         training, the users could set `split_thr` to a small value.
    ///         If the number of boxes is greater than the threshold, it will
    ///         perform NMS on each group of boxes separately and sequentially.
    ///         Defaults to 10000.</param>
    /// <param name="class_agnostic">if true, nms is class agnostic,
    ///     i.e. IoU thresholding happens over all boxes,
    ///     regardless of the predicted class.</param>
    /// <returns>tuple: kept dets and indice.</returns>
    static std::tuple<torch::Tensor, torch::Tensor> batched_nms(
        torch::Tensor boxes,
        torch::Tensor scores,
        torch::Tensor rows,
        float iou_thre,
        torch::optional<int> split_thr = 10000,
        torch::optional<bool> class_agnostic = false) {

        if (!split_thr) {
            split_thr = 10000;
        }
        if (!class_agnostic) {
            class_agnostic = false;
        }

        torch::Tensor boxes_for_nms;
        if (class_agnostic && class_agnostic.value()) {
            boxes_for_nms = boxes;
        } else {
            auto max_coordinate = boxes.max().item().toFloat();
            auto offsets = rows.to(boxes) * (max_coordinate + 1);
            boxes_for_nms = boxes + offsets.view({-1, 1});
        }

        torch::Tensor keep;
        if (split_thr && boxes_for_nms.size(0) < split_thr.value()) {
            keep = vision::ops::nms(boxes_for_nms, scores, iou_thre);
            boxes = boxes.index_select(0, keep);
            scores = scores.index_select(0, keep);
        } else {
            auto total_mask = scores.new_zeros(scores.sizes(), c10::TensorOptions().dtype(torch::kBool));

            auto maxidx = rows.max().item().toLong();
            for (int id = 0; id < maxidx; ++id) {
                auto mask = (rows == id).nonzero().view(-1);
                // dets, keep = nms_op(boxes_for_nms[mask], scores[mask], **nms_cfg_);
                auto keep = vision::ops::nms(boxes_for_nms[mask], scores[mask], iou_thre);
                total_mask.masked_select(mask.index_select(0, keep)) = true;
            }

            keep = total_mask.nonzero().view(-1);
            keep = keep.index_select(0, scores.index_select(0, keep).argsort(-1, true));
            boxes = boxes.index_select(0, keep);
            scores = scores.index_select(0, keep);
        }

        return {torch::cat({boxes, scores.unsqueeze(1)}, -1), keep};
    }

    /// <summary>
    /// NMS for multi-class bboxes.
    /// </summary>
    /// <param name="multi_bboxes">shape (n, #class*4) or (n, 4)</param>
    /// <param name="multi_scores">shape (n, #class), where the last column contains scores of the background class, but this will be ignored.</param>
    /// <param name="score_thr">bbox threshold, bboxes with scores lower than it will not be considered.</param>
    /// <param name="nms_thr">NMS IoU threshold</param>
    /// <param name="max_num">if there are more than max_num bboxes after NMS, only top max_num will be kept.</param>
    /// <param name="score_factors">The factors multiplied to scores before applying NMS</param>
    /// <returns> (k, 6) tensor as (bboxes, labels), tensors of shape (k, 5) and (k, 1). Labels are 0-based.</returns>
    static torch::Tensor multiclass_nms(
        torch::Tensor multi_bboxes,
        torch::Tensor multi_scores,
        float score_thre,
        float iou_thre,
        int max_num = -1,
        torch::optional<int> split_thr = torch::nullopt,
        torch::optional<bool> class_agnostic = torch::nullopt,
        torch::optional<torch::Tensor> score_factors = torch::nullopt) {

        int num_classes = multi_scores.size(1) - 1;

        // exclude background category
        torch::Tensor bboxes, scores;
        if (multi_bboxes.size(1) > 4) {
            bboxes = multi_bboxes.view({multi_scores.size(0), -1, 4});
        } else {
            bboxes = multi_bboxes.unsqueeze(1).expand({multi_scores.size(0), num_classes, 4});
            scores = multi_scores.index({Slice(None), Slice(None, -1)});
        }

        // filter out boxes with low scores
        // bboxes: (N, 12, 4), scores: (N, 12)
        auto valid_mask = scores > score_thre;  // (N, 12)
        bboxes = bboxes.masked_select(valid_mask.unsqueeze(-1).repeat({1, 1, 4})).view({-1, 4});
        if (score_factors) {
            scores = scores * score_factors.value().unsqueeze(1);
        }
        scores = scores.masked_select(valid_mask);
        auto labels = valid_mask.nonzero().slice(1, 1);  // nonzero: (N, 12) -> (N', 2)/(r, c), select only row index.
        if (bboxes.numel() == 0) {
            return multi_bboxes.new_zeros({0, 6});
        }

        // bboxes: (N, 4), scores: (N), labels: (N)
        torch::Tensor dets, keep;
        std::tie(dets, keep) = batched_nms(bboxes, scores, labels, iou_thre, split_thr, class_agnostic);

        if (max_num > 0) {
            dets = dets.slice(0, 0, max_num);
            keep = keep.slice(0, 0, max_num);
        }

        auto cls = labels.index_select(0, keep).to(dets);
        return torch::cat({dets, cls}, 1);
    }

}  // namespace nanodet
